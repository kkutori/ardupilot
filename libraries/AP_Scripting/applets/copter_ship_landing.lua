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
  // @Param: SHIP_WP_OFS_X
  // @DisplayName: offset x of copter to home
  // @Description: positive: copter is to the front of the home, negative: copter is to the back of the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_WP_OFS_X = bind_add_param('WP_OFS_X', 3, -20)

--[[
  // @Param: SHIP_WP_OFS_Y
  // @DisplayName: offset y of copter to home
  // @Description: positive: copter is to the right of the home, negative: copter is to the left of the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_WP_OFS_Y = bind_add_param('WP_OFS_Y', 4, 0)

--[[
  // @Param: SHIP_WP_OFS_Z
  // @DisplayName: offset altitude of copter to home
  // @Description: positive: copter is always above the home, negative: copter is above the home
  // @Range: 
  // @Units: meter
  // @User: Standard
--]]
SHIP_WP_OFS_Z = bind_add_param('WP_OFS_Z', 5, -30)



-- other parameters
RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
RTL_ALT         = bind_param("RTL_ALT") -- uint: cm, range:200 to 300000
-- these 3 parameters below are uased to set target offset which will be set home
-- the offsets of the copter to the home rely on ship_wp_ofs_*
FOLL_OFS_X      = bind_param("FOLL_OFS_X")
FOLL_OFS_Y      = bind_param("FOLL_OFS_Y")
FOLL_OFS_Z      = bind_param("FOLL_OFS_Z")

-- an auth ID to disallow arming when we don't have the beacon
local auth_id = arming:get_aux_auth_id()
arming:set_aux_auth_failed(auth_id, "Ship: no abeacon")

-- target in follow mode, here can been seen as ship or home
local target_pos = Location()
local target_velocity = Vector3f()
-- target's heading in degrees (0 = north, 90 = east)
local target_heading = 0.0
-- positon where the aircraft should fly to
local desired_pos = Location()
-- The actual indicated location 
local next_wp = Location()
-- current copter position
local current_pos = Location()

-- landing stages
local STAGE_IDLE = 0
local STAGE_FOLLOW = 1
local STAGE_CLIMB = 2
local STAGE_APPROACH = 3
local STAGE_LAND = 4
local landing_stage = STAGE_IDLE

-- other state
local vehicle_mode = MODE_LOITER
local throttle_pos = THROTTLE_HIGH
local have_target = false

-- dynamic compensation
local COMPENSATION = 0
local COMPENSATION_HOME = 0
local distance_history = {} -- Store history of distances
local DISTANCE_HISTORY_SIZE = 10  -- Number of distance measurements to keep
local RECORD_CNT = 0
local WAITING_CNT = 0
local WAITING_STABILITY = 0
local DISTANCE_STABILITY_THRESHOLD = 0.15  -- Meters, threshold for considering distance stable
local COMPENSATION_STEP = 0.2  -- adjust compensation each time
local MAX_COMPENSATION = 20  -- Maximum compensation value

-- landing parameters
local landing_detected_counter = 0
local LANDING_CONFIRM_COUNT = 10
local LANDED = false
local LANDING_IMPACT_DETECTED = false

GRAVITY_MSS = 9.80665

local dist_hor = 0
local dist_hor_prev = 0



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

   if mode == MODE_RTL then 
      vehicle:set_mode(MODE_GUIDED)
      mode = MODE_GUIDED
      landing_stage = STAGE_FOLLOW
   end

   vehicle_mode = mode
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

   -- param:set('FOLL_DIST_MAX',1000)

   target_pos, target_velocity = follow:get_target_location_and_velocity_ofs()
   target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
   target_heading = follow:get_target_heading_deg()
   -- zero vertical velocity to reduce impact of ship movement
   target_velocity:z(0)
end

--[[
   during the actual flight, copter will lag behind the next_wp by a certain distance, so a certain distance compensation should be given
--]]
function update_compensation()
   -- lua run at 20hz, run every 50 ms

   -- compensation is new, wait aircraft fly away from last stable position
   if WAITING_CNT > 0 and WAITING_CNT < 30 then 
      WAITING_CNT = WAITING_CNT + 1
      return
   else 
      WAITING_CNT = 0
   end

   -- record data every 500 ms
   -- analysis 10 datas in 5 seconds
   if RECORD_CNT <= 10 then
      RECORD_CNT = RECORD_CNT + 1
      return
   else
      RECORD_CNT = 0
   end


   local new_dist = current_pos:get_distance(desired_pos)
   table.insert(distance_history, new_dist)
   if #distance_history > DISTANCE_HISTORY_SIZE then
      table.remove(distance_history, 1)
   end

   if #distance_history < DISTANCE_HISTORY_SIZE then
      return
   end

   local stable = true
   local avg_dist = 0

   for i , dist in ipairs(distance_history) do
      avg_dist = avg_dist + dist
   end
   avg_dist = avg_dist / #distance_history

   for i, dist in ipairs(distance_history) do
      if math.abs(avg_dist - dist) > DISTANCE_STABILITY_THRESHOLD then
         stable = false
         break
      end
   end

   -- compensation is new, aircraft will fly to new desired position, wait stability
   if WAITING_STABILITY == 1 and stable == true then 
      WAITING_STABILITY = 0
   end

   -- no need to wait aircraft fly to new desired position
   if WAITING_STABILITY ~= 1 and stable ~= true then
      COMPENSATION = 0
   end

   -- if copter is close to the ship, do not update compensation
   if stable and avg_dist > 0.3 then
      -- judge if current_pos is behind desired_pos
      -- Calculate vector from desired_pos to current_pos
      local vec_to_current = desired_pos:get_distance_NED(current_pos)
      -- Transform vector to coordinate system relative to target heading
      local heading_rad = math.rad(target_heading)
      local cos_heading = math.cos(heading_rad)
      local sin_heading = math.sin(heading_rad)
      -- Rotate coordinate system to align x-axis with target heading
      local x_rotated = vec_to_current:x() * cos_heading + vec_to_current:y() * sin_heading
      -- Tolerance value in meters
      local tolerance = 0.1
      -- Determine front/back position relationship with tolerance
      if x_rotated > tolerance then
         COMPENSATION = COMPENSATION - avg_dist + COMPENSATION_STEP
      elseif x_rotated < -tolerance then
         COMPENSATION = COMPENSATION + avg_dist
      else
         return
      end

      COMPENSATION = math.min(COMPENSATION, MAX_COMPENSATION)


      WAITING_CNT = 1
      WAITING_STABILITY = 1
      distance_history = {}
      -- debug
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("new comp %.2fm, avgdist: %.2fm", COMPENSATION, avg_dist))
   end
end


local timestamp1 = 0 -- first time reach the position
local timestamp2 = 0 -- current time at the position

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
   local copter_above_home_alt = current_pos:alt() - target_pos:alt() -- cm
   -- local copter_alt = current_pos:alt() -- cm
   -- local home_alt = target_pos:alt() -- cm
   local distance_alt = math.abs(copter_above_home_alt - RTL_ALT:get()) -- centimeters

   -- 2. check lat lon
   local distance_xy = current_pos:get_distance(target_pos) -- meters

   if landing_stage == STAGE_APPROACH then
      -- altitude within 10 cm, horizontal within 0.2 m
      if distance_xy <= 0.2 then
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

   elseif landing_stage == STAGE_LAND then
      -- horizontal within 0.2 m
      if distance_xy <= 1.0 then
         return true
      else
         return false
      end
   end
end


--[[
  check if copter has landed
--]]
function check_landed()
   local vel_z = ahrs:get_velocity_NED():z()
   local acc_z = ahrs:get_accel():z()

   -- detect impact
   if math.abs(acc_z) - GRAVITY_MSS > 3 then
      LANDING_IMPACT_DETECTED = true
      -- debug
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("accz:%.2f", acc_z))
   end


   if LANDING_IMPACT_DETECTED then
      if math.abs(vel_z) < 0.2 then -- <20 cm/s
         landing_detected_counter = landing_detected_counter + 1
      else
         landing_detected_counter = 0 -- vel_z too high, reset counter
      end
   else
      landing_detected_counter = 0 -- flying, reset counter
   end

   if landing_detected_counter >= LANDING_CONFIRM_COUNT then
      LANDING_IMPACT_DETECTED = false
      landing_detected_counter = 0
      return true
   end


   -- if impact not detected, check compensation
   -- if aircraft has landed but not been recognized, the compensation will continue to increase, but distance from home will not change.
   if not LANDING_IMPACT_DETECTED then
      if COMPENSATION - COMPENSATION_HOME > 1 and COMPENSATION_HOME ~= 0 then 
         dist_hor = current_pos:get_distance(target_pos) -- meters
         -- compensation is bigger than the value got when aircraft was derectly above home
         -- but aircraft is colse to home, so aircraft didn't fly away, aka aircraft has landed
         if dist_hor < 1 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("CL dist:%.2f, com:%.2f, comh:%.2f", dist_hor, COMPENSATION, COMPENSATION_HOME))
            COMPENSATION_HOME = 0
            return true
         end
      end
   end


   return false
end

--[[
  adjust landing speed 
--]]
function land_speed_control()
   local alt_temp = current_pos:alt() - target_pos:alt()
   if alt_temp < 500 then
      if param:get("WPNAV_SPEED_DN") > 15 then
         gcs:send_text(MAV_SEVERITY.INFO, "Slow down 15cm/s")
         param:set("WPNAV_SPEED_DN", 15) -- cm/s
      end
   elseif alt_temp < 1000 then
      if param:get("WPNAV_SPEED_DN") > 50 then
         gcs:send_text(MAV_SEVERITY.INFO, "Slow down 50cm/s")
         param:set("WPNAV_SPEED_DN", 50) -- cm/s
      end
   end
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

   -- if not armed, return
   if not arming:is_armed() then 
      if COMPENSATION ~= 0 then
         COMPENSATION = 0
      end
      return
   end

   -- check throttle position to cntrol landing stages
   --[[
      HIGH: >= 40% throttle
      MID:  >= 10% throttle
      LOW:  < 10%  throttle
   --]]
   update_throttle_pos()

   -- get vehicle mode (RTL?)
   update_mode()

   -- copter should fly to next_wp
   next_wp = target_pos:copy()
   desired_pos = target_pos:copy()

   if vehicle_mode == MODE_GUIDED then
      if landing_stage == STAGE_FOLLOW then 
         -- update desired_pos with offset
         local heading_rad = math.rad(target_heading)
         local N_offset = SHIP_WP_OFS_X:get() * math.cos(heading_rad) - SHIP_WP_OFS_Y:get() * math.sin(heading_rad)
         local E_offset = SHIP_WP_OFS_X:get() * math.sin(heading_rad) + SHIP_WP_OFS_Y:get() * math.cos(heading_rad)
         desired_pos:offset(N_offset, E_offset)

         -- during the actual flight, copter will lag behind the next_wp by a certain distance, so a certain distance compensation should be given
         update_compensation()

         -- update next_wp with offset and compensation
         N_offset = (SHIP_WP_OFS_X:get() + COMPENSATION) * math.cos(heading_rad) - SHIP_WP_OFS_Y:get() * math.sin(heading_rad)
         E_offset = (SHIP_WP_OFS_X:get() + COMPENSATION) * math.sin(heading_rad) + SHIP_WP_OFS_Y:get() * math.cos(heading_rad)
         next_wp:offset(N_offset, E_offset)
         next_wp:alt(target_pos:alt() - SHIP_WP_OFS_Z:get()*100) -- uintt: cm m

         vehicle:set_target_location(next_wp)

         -- check if copter should descend and fly directly above home
         if throttle_pos == THROTTLE_LOW then
            if (current_pos:alt() - target_pos:alt()) < RTL_ALT:get() then
               landing_stage = STAGE_CLIMB
            else
               landing_stage = STAGE_APPROACH
            end
         end

      elseif landing_stage == STAGE_CLIMB then
         -- update desired_pos with offset
         local heading_rad = math.rad(target_heading)
         local N_offset = SHIP_WP_OFS_X:get() * math.cos(heading_rad) - SHIP_WP_OFS_Y:get() * math.sin(heading_rad)
         local E_offset = SHIP_WP_OFS_X:get() * math.sin(heading_rad) + SHIP_WP_OFS_Y:get() * math.cos(heading_rad)
         desired_pos:offset(N_offset, E_offset)

         -- update compensation
         update_compensation()

         -- update next_wp with offset and compensation
         N_offset = (SHIP_WP_OFS_X:get() + COMPENSATION) * math.cos(heading_rad) - SHIP_WP_OFS_Y:get() * math.sin(heading_rad)
         E_offset = (SHIP_WP_OFS_X:get() + COMPENSATION) * math.sin(heading_rad) + SHIP_WP_OFS_Y:get() * math.cos(heading_rad)
         next_wp:offset(N_offset, E_offset)
         next_wp:alt(target_pos:alt() + RTL_ALT:get()) -- uintt: cm cm

         vehicle:set_target_location(next_wp)

         -- check if get desired altitude
         if reached_altitude() then
            landing_stage = STAGE_APPROACH
            gcs:send_text(MAV_SEVERITY.INFO, "Climbed to RTL_ALT")
         end

         -- check throttle position
         if throttle_pos ~= THROTTLE_LOW then
            landing_stage = STAGE_FOLLOW
         end

      elseif landing_stage == STAGE_APPROACH then
         -- desired_pos no offset

         -- update_compensation
         update_compensation()

         -- update next_wp with compensation
         local heading_rad = math.rad(target_heading)
         local N_offset = COMPENSATION * math.cos(heading_rad)
         local E_offset = COMPENSATION * math.sin(heading_rad)
         next_wp:offset(N_offset, E_offset)
         next_wp:alt(target_pos:alt()+ RTL_ALT:get()) -- uint: cm cm

         vehicle:set_target_location(next_wp)

         -- check if copter is directly above home
         if is_directly_above_home() then
            gcs:send_text(MAV_SEVERITY.INFO, "Aircraft is directly above home, start landing")
            landing_stage = STAGE_LAND
            COMPENSATION_HOME = COMPENSATION
         end

         -- check throttle position
         if throttle_pos ~= THROTTLE_LOW then
            landing_stage = STAGE_FOLLOW
            -- land on this waypoint, do not reset compensation
            -- COMPENSATION = 0
         end

      elseif landing_stage == STAGE_LAND then
         -- update compensation
         update_compensation()

         -- update next_wp with compensation
         local heading_rad = math.rad(target_heading)
         local north_offset = COMPENSATION * math.cos(heading_rad)
         local east_offset = COMPENSATION * math.sin(heading_rad)
         next_wp:offset(north_offset, east_offset)

         -- check if copter is directly above home
         if is_directly_above_home() then
            -- a little below home
            next_wp:alt(target_pos:alt() - 50) -- uint: cm cm
         else
            -- wait fly to directly above home at current altitude
            next_wp:alt(current_pos:alt()) -- uint: cm
         end

         vehicle:set_target_location(next_wp)

         -- speed control
         land_speed_control()

         -- check if landed
         LANDED = check_landed()

         -- check if abort landing
         if throttle_pos ~= THROTTLE_LOW then
            gcs:send_text(MAV_SEVERITY.INFO, "Throttle high, abort landing")
            landing_stage = STAGE_IDLE
            vehicle:set_mode(MODE_LOITER)
            vehicle_mode = MODE_LOITER
            COMPENSATION = 0
            param:set("WPNAV_SPEED_DN", 150) -- cm/s
         end
      end

   else 
      -- if rtl, lua will change vehicle_mode to guided immediately
      -- in other modes, reset landing_stage idle
      landing_stage = STAGE_IDLE
   end

   -- landed, disarm motors, reset flags
   if LANDED then
      COMPENSATION = 0
      landing_stage = STAGE_IDLE
      vehicle:set_mode(MODE_STABILIZE)
      vehicle_mode = MODE_STABILIZE
      param:set("WPNAV_SPEED_DN", 150) -- cm/s
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
