--[[
 support takeoff and landing on moving platforms for Copter

 This script enables a copter to land on a moving ship by:
 1. Following the ship at a safe altitude when in RTL mode
 2. Initiating descent when throttle stick is lowered
 3. Landing directly on the ship when conditions are met

 Based on plane_ship_landing.lua for VTOL planes
--]]

local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- local mavlink_msgs = require("/home/zjl/ardupilot/libraries/AP_HAL_ChibiOS/hwdef/JCFH-A1/scripts/modules/MAVLink/mavlink_msgs")

-- local msg_map = {}
-- local landing_target_msgid = mavlink_msgs.get_msgid("LANDING_TARGET")
-- msg_map[landing_target_msgid] = "LANDING_TARGET"
-- -- initialise mavlink rx with number of messages, and buffer depth
-- mavlink.init(1, 10)
-- -- register message id to receive
-- mavlink.register_rx_msgid(landing_target_msgid)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 8
local PARAM_TABLE_PREFIX = "SHIP_"

-- local MODE_MANUAL = 0
local MODE_STABILIZE = 0
local MODE_GUIDED = 4
local MODE_RTL = 6
local MODE_LOITER = 5
local MODE_AUTO = 3
local MODE_LAND = 9
local MODE_FOLLOW = 23

local ALT_FRAME_ABSOLUTE = 0

-- 3 throttle position
local THROTTLE_LOW = 0
local THROTTLE_MID = 1
local THROTTLE_HIGH = 2

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup SHIP specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')
--[[
  // @Param: SHIP_ENABLE
  // @DisplayName: Copter ship landing enable
  // @Description: Enable copter ship landing system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
SHIP_ENABLE     = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: SHIP_AUTO_OFS
  // @DisplayName: Copter ship automatic offset trigger
  // @Description: Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target. NOTE: This parameter will auto-reset to zero once the offset has been calculated.
  // @Values: 0:Disabled,1:Trigger
  // @User: Standard
--]]
SHIP_AUTO_OFS   = bind_add_param('AUTO_OFS', 2, 0)


--[[
  // @Param: SHIP_FL_X
  // @DisplayName: offset x of copter to home
  // @Description: positive: copter is to the front of the home, negative: copter is to the back of the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_FL_X = bind_add_param('FL_X', 3, -20)

--[[
  // @Param: SHIP_FL_Y
  // @DisplayName: offset y of copter to home
  // @Description: positive: copter is to the right of the home, negative: copter is to the left of the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_FL_Y = bind_add_param('FL_Y', 4, 0)

--[[
  // @Param: SHIP_FL_Z
  // @DisplayName: offset altitude of copter to home
  // @Description: positive: copter is always above the home, negative: copter is above the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_FL_Z = bind_add_param('FL_Z', 5, -30)



-- other parameters
RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
RTL_ALT         = bind_param("RTL_ALT") -- uint: cm, range:200 to 300000
-- these 3 parameters below are uased to set target offset which will be set home
-- the offsets of the copter to the home rely on SHIP_FL_*
FOLL_OFS_X      = bind_param("FOLL_OFS_X")
FOLL_OFS_Y      = bind_param("FOLL_OFS_Y")
FOLL_OFS_Z      = bind_param("FOLL_OFS_Z")

-- an auth ID to disallow arming when we don't have the beacon
local auth_id = arming:get_aux_auth_id()
arming:set_aux_auth_failed(auth_id, "Ship: no abeacon")

-- target in follow mode, here it can been seen as ship or home
local target_pos = Location()
local target_velocity = Vector3f()
-- target heading in degrees (0 = north, 90 = east)
local target_heading = 0.0
-- current copter position
local current_pos = Location()
-- positon where the aircraft should fly to
local desired_pos = Location()
-- the expected velocity of the drone
local desired_vel = Vector3f()

-- climb position offset
local climb_offset = Vector2f()
local CLIMB_OFFSET_X = 0
local CLIMB_OFFSET_Y = 0

-- landing
local STAGE_IDLE = 0
local STAGE_FOLLOW = 1
local STAGE_CLIMB = 2
local STAGE_APPROACH = 3
local STAGE_LAND = 4
local landing_stage = STAGE_IDLE
local LANDED = false

-- other state
local vehicle_mode = MODE_STABILIZE
local throttle_pos = THROTTLE_HIGH
local have_target = false
local VEL_PROP = 0.7

-- time stamp
local timestamp1 = 0 -- first time reach the position
local timestamp2 = 0 -- current time at the position

local dist_horizontal = 0
local dist_horizontal_prev = 0


-- check key parameters
function check_parameters()
   --[[
      parameter values which are auto-set on startup
   --]]
   local key_params = {
      FOLL_ENABLE = 1,
      FOLL_OFS_TYPE = 1,
      FOLL_ALT_TYPE = 0,
      -- FOLL_DIST_MAX = 1000,
   }

   for p, v in pairs(key_params) do
      local current = param:get(p)
      assert(current, string.format("Parameter %s not found", p))
      if math.abs(v-current) > 0.001 then
         param:set_and_save(p, v)
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Parameter %s set to %.2f was %.2f", p, v, current))
      end
   end
end

-- update the pilots throttle position
function update_throttle_pos()
   local tpos
   if not rc:has_valid_input() then
      tpos = THROTTLE_LOW
   else
      local tchan = rc:get_channel(RCMAP_THROTTLE:get())
      local tval = (tchan:norm_input_ignore_trim()+1.0)*0.5
      if tval >= 0.40 then
         tpos = THROTTLE_HIGH
      elseif tval >= 0.1 then
         tpos = THROTTLE_MID
      else
         tpos = THROTTLE_LOW
      end
   end

   throttle_pos = tpos
end


-- update state based on vehicle mode
function update_mode()
   local mode = vehicle:get_mode()
   if mode == vehicle_mode then
      return
   end
   vehicle_mode = mode

   if mode == MODE_RTL then 
      vehicle:set_mode(MODE_GUIDED)
      vehicle_mode = MODE_GUIDED
      landing_stage = STAGE_FOLLOW
   else
      landing_stage = STAGE_IDLE
   end
end

-- update target state, 
-- the target_pos is somewhere the copter will fly to in follow mode
-- here it can been seen as ship or home
function update_target()
   if not follow:have_target() then
      if have_target then
         gcs:send_text(MAV_SEVERITY.WARNING, "Lost beacon")
         arming:set_aux_auth_failed(auth_id, "Ship: no bbeacon")
      else
         gcs:send_text(MAV_SEVERITY.INFO, "Found beacon")
      end
      have_target = false
      return
   end
   if not have_target then
      gcs:send_text(MAV_SEVERITY.INFO, "Have beacon")
      arming:set_aux_auth_passed(auth_id)
   end
   have_target = true

   target_pos, target_velocity = follow:get_target_location_and_velocity_ofs()
   target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   target_heading = follow:get_target_heading_deg()
   -- zero vertical velocity to reduce impact of ship movement
   target_velocity:z(0)
end


--[[
  check if we've reached desired altitude
--]]
function reached_altitude()
   if math.abs(current_pos:alt() - target_pos:alt() - RTL_ALT:get()) < 20 then
      if timestamp1 == 0 then
         -- record first time aircraft reach landing altitude
         timestamp1 = millis():toint()
      end
      -- record current time
      timestamp2 = millis():toint()

      -- if time difference is more than 3 seconds, aircraft is stable
      if timestamp2 - timestamp1 > 3000 and timestamp1 ~= 0 then
         timestamp1 = 0
         return true
      end
   else 
      timestamp1 = 0
   end

   return false
end

--[[
  check if we've reached directly above home
--]]
function is_directly_above_home()
   -- check if copter is directly above home
   -- 1. check alt
   local alt_diff = math.abs(current_pos:alt() - target_pos:alt() - RTL_ALT:get()) -- centimeters

   -- 2. check lat lon
   dist_horizontal = current_pos:get_distance(target_pos) -- meters

   -- altitude within 20 cm, horizontal within 0.2 m
   if alt_diff <= 20 and dist_horizontal <= 0.2 then
      -- check stable
      -- record first time aircraft reach directly above home
      if timestamp1 == 0 then
         timestamp1 = millis():toint()
      end
      -- record current time
      timestamp2 = millis():toint()

      -- if time difference is more than 3 seconds, aircraft is stable
      if timestamp2 - timestamp1 > 3000 and timestamp1 ~= 0 then
         timestamp1 = 0
         return true
      end
   else
      timestamp1 = 0
   end

   return false
end


--[[
  check if copter has landed
--]]
function check_landed()
   local vel_z = ahrs:get_velocity_NED():z()
   local height = current_pos:alt() - target_pos:alt()
   dist_horizontal = current_pos:get_distance(target_pos)

   if vel_z < 0.1 and math.abs(height) < 50 then 
      if timestamp1 ~= 0 then
         timestamp1 = millis():toint()
      end
      timestamp2 = millis():toint()

      if timestamp2 - timestamp1 > 4000 then
         -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Land, dist_diff:%.2f", dist_horizontal))
         return true
      end
   end

   return false
end


--[[
  adjust landing speed 
--]]
function set_aircraft_velocity()
   local rel_pos = Vector3f()
   rel_pos = current_pos:get_distance_NED(desired_pos)

   -- calculate correction
   local correction_vel = {
      x = VEL_PROP * rel_pos:x(),
      y = VEL_PROP * rel_pos:y(),
      z = VEL_PROP * rel_pos:z()
   }

   -- limit velocity
   local correction_mag = math.sqrt(correction_vel.x^2 + correction_vel.y^2 + correction_vel.z^2)
   if correction_mag > 1.0 then
      local scale = 1.0 / correction_mag
      correction_vel.x = correction_vel.x * scale
      correction_vel.y = correction_vel.y * scale
      correction_vel.z = correction_vel.z * scale
   end

   -- aircraft_velocity = ship_velocity + correction
   desired_vel:x(target_velocity:x() + correction_vel.x)
   desired_vel:y(target_velocity:y() + correction_vel.y)

   if landing_stage == STAGE_LAND then
      if (current_pos:alt() - target_pos:alt() > 500) then
         desired_vel:z(0.6)
      else
         desired_vel:z(0.3)
      end
   elseif landing_stage == STAGE_CLIMB then
      if ((current_pos:alt() - target_pos:alt()) < (RTL_ALT:get() - 50)) then
         desired_vel:z(-0.8)
      else
         desired_vel:z(target_velocity:z() + correction_vel.z)
      end
   else
         desired_vel:z(target_velocity:z() + correction_vel.z)
   end

   vehicle:set_target_velocity_NED(desired_vel)
end

--[[
  update automatic beacon offsets
--]]

function update_auto_offset()
   if arming:is_armed() or math.floor(SHIP_AUTO_OFS:get()) ~= 1 then
      return
   end

   -- get target without offsets applied
   target_no_ofs, vel = follow:get_target_location_and_velocity()
   target_no_ofs:change_alt_frame(ALT_FRAME_ABSOLUTE)

   -- setup offsets so target location will be current location
   local new = target_no_ofs:get_distance_NED(current_pos)
   new:rotate_xy(-math.rad(target_heading))

   gcs:send_text(MAV_SEVERITY.INFO, string.format("Set follow offset (%.2f,%.2f,%.2f)", new:x(), new:y(), new:z()))
   FOLL_OFS_X:set_and_save(new:x())
   FOLL_OFS_Y:set_and_save(new:y())
   FOLL_OFS_Z:set_and_save(new:z())

   SHIP_AUTO_OFS:set_and_save(0)
end

-- main update function
function update()
   if SHIP_ENABLE:get() < 1 then
      return
   end

   -- target means ship/home
   update_target()
   if not have_target then
      return
   end

   ahrs:set_home(target_pos)

   -- get vehicle position
   current_pos = ahrs:get_position()
   if not current_pos then
      return
   end
   current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

   --set follow offset
   update_auto_offset()

   -- check throttle position to cntrol landing stages
   --[[
      HIGH:[40,100], MID:[10,40), LOW:9[0,10)
   --]]
   update_throttle_pos()

   -- get vehicle mode
   update_mode()

   desired_pos = target_pos:copy()

   if vehicle_mode == MODE_GUIDED then
      if landing_stage == STAGE_FOLLOW then 
         -- update desired_pos with offset
         local heading_rad = math.rad(target_heading)
         local N_offset = SHIP_FL_X:get() * math.cos(heading_rad) - SHIP_FL_Y:get() * math.sin(heading_rad)
         local E_offset = SHIP_FL_X:get() * math.sin(heading_rad) + SHIP_FL_Y:get() * math.cos(heading_rad)
         desired_pos:offset(N_offset, E_offset)
         desired_pos:alt(target_pos:alt() - SHIP_FL_Z:get()*100) -- uintt: cm m

         dist_horizontal = current_pos:get_distance(desired_pos) -- meters
         if dist_horizontal < 20 then
            set_aircraft_velocity()
         else
            vehicle:set_target_location(desired_pos)
         end

         -- check if copter should climb first or fly directly above home
         if throttle_pos == THROTTLE_LOW or throttle_pos == THROTTLE_MID then
            if (current_pos:alt() - target_pos:alt()) < RTL_ALT:get() then
               landing_stage = STAGE_CLIMB
               -- remember the offset and climb at this relative position
               climb_offset = target_pos:get_distance_NE(current_pos)
               CLIMB_OFFSET_X = climb_offset:x() * math.cos(heading_rad) + climb_offset:y() * math.sin(heading_rad)
               CLIMB_OFFSET_Y = -climb_offset:x() * math.sin(heading_rad) + climb_offset:y() * math.cos(heading_rad)
               gcs:send_text(MAV_SEVERITY.INFO, string.format("Start climbing"))
            else
               landing_stage = STAGE_APPROACH
            end
         end

      elseif landing_stage == STAGE_CLIMB then
         -- update desired_pos with offset
         local heading_rad = math.rad(target_heading)
         local N_offset = CLIMB_OFFSET_X * math.cos(heading_rad) - CLIMB_OFFSET_Y * math.sin(heading_rad)
         local E_offset = CLIMB_OFFSET_X * math.sin(heading_rad) + CLIMB_OFFSET_Y * math.cos(heading_rad)
         desired_pos:offset(N_offset, E_offset)
         desired_pos:alt(target_pos:alt() + RTL_ALT:get())

         set_aircraft_velocity()

         -- check if get desired altitude
         if reached_altitude() then
            landing_stage = STAGE_APPROACH
            gcs:send_text(MAV_SEVERITY.INFO, "Climbed to RTL_ALT")
         end

         -- check throttle position
         if throttle_pos == THROTTLE_HIGH then
            landing_stage = STAGE_FOLLOW
         end

      elseif landing_stage == STAGE_APPROACH then
         -- desired_pos no offset in x y
         desired_pos:alt(target_pos:alt() + RTL_ALT:get())

         dist_horizontal = current_pos:get_distance(desired_pos) -- meters
         if dist_horizontal < 20 then
            set_aircraft_velocity()
         else
            vehicle:set_target_location(desired_pos)
         end

         -- THROTTLE_LOW and aircraft is directly above home, go to STAGE_LAND
         if is_directly_above_home() and throttle_pos == THROTTLE_LOW then
            gcs:send_text(MAV_SEVERITY.INFO, "Aircraft is directly above home, start landing")
            landing_stage = STAGE_LAND
         end

         -- THROTTLE_MID, (1)holdoff, (2)climb to RTL_ALT when quit STAGE_LAND

         -- THROTTLE_HIGH, go to STAGE_FOLLOW
         -- 3 ways go to STAGE_FOLLOW, (1)from STAGE_FOLLOW, (2)from STAGE_CLIMB, (3)from STAGE_LAND
         -- only in (3), aircraft_alt will be lower than ship_alt+RTL_ALT
         if throttle_pos == THROTTLE_HIGH and (current_pos:alt() - target_pos:alt() - RTL_ALT:get()) > -20 then
            landing_stage = STAGE_FOLLOW
         end

      elseif landing_stage == STAGE_LAND then
         set_aircraft_velocity()
         -- check if copter has landed
         LANDED = check_landed()

         -- check if abort landing
         if throttle_pos ~= THROTTLE_LOW then
            gcs:send_text(MAV_SEVERITY.INFO, "Throttle HIGH/MID, abort landing, climb to RTL_ALT")
            landing_stage = STAGE_APPROACH
         end
      end
   end

   -- landed, disarm motors, reset flags
   if LANDED then
      landing_stage = STAGE_IDLE
      vehicle:set_mode(MODE_STABILIZE)
      vehicle_mode = MODE_STABILIZE
      if arming:disarm() then
         gcs:send_text(MAV_SEVERITY.INFO, "Motors disarmed successfully.")
         LANDED = false
      else
         gcs:send_text(MAV_SEVERITY.INFO, "Disarm command failed. Retrying.")
      end
   end

end

function loop()
   update()
   -- run at 20Hz
   return loop, 50
end

check_parameters()

gcs:send_text(MAV_SEVERITY.INFO, "ShipLanding: loaded")

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

-- start running update loop
return protected_wrapper()
