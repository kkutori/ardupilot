--[[
  串口通信模块脚本
  功能：延迟5秒启动脚本，读取6个温度数据
--]]

--[[
  
--]]

-- 定义MAV_SEVERITY常量表
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}


local PARAM_TABLE_KEY = 99  -- 参数表键值
local PARAM_TABLE_PREFIX = "MOTOR_TMP_"  -- 参数前缀

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

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not add param table')
MOTOR_TEMP_LOG_EN = bind_add_param('LOG_EN', 1, 0)  -- 0:disable, 1:enable
-- MOTOR_TEMP_ABC   = bind_add_param('ABC', 2, 0)



-- 配置参数
local SERIAL_PORT = 1     -- 串口端口号，对应SERIALx_PROTOCOL = 28的端口
local BAUD_RATE = 115200  -- 波特率
local UPDATE_RATE_MS = 1000 -- 更新频率(毫秒)
local MAX_BYTES_TO_READ = 20 -- 每次读取的最大字节数
local ENABLE_DEBUG = false -- 启用调试信息
local START_DELAY_SEC = 5 -- 启动延迟时间(秒)

-- 全局变量
local MSG_HEADER = 0xAA  -- 消息头标识符
local MSG_ERR = -99      -- 错误代码
local motor_temperature = {0, 0, 0, 0, 0, 0}  -- range -128 to 127, temperature values divided by 100
local port = nil
local initialized = false -- 初始化状态
local delay_timer = 0     -- 延迟计时器
local delay_complete = false -- 延迟完成状态


-- 查找并初始化串口
function init_serial()
  -- 查找脚本串口实例 (0索引，所以减1)
  port = serial:find_serial(SERIAL_PORT - 1)
  if not port then
    gcs:send_text(MAV_SEVERITY.ERROR, "Init Fail")
    return false
  end
  
  -- 初始化串口
  port:begin(BAUD_RATE)
  
  return true
end

-- 获取温度数据函数（供外部调用）
function get_temperature_data()
    return motor_temperature[1], motor_temperature[2], motor_temperature[3],
           motor_temperature[4], motor_temperature[5], motor_temperature[6]
end

-- 处理接收到的数据
function process_received_data(data)
    -- 检查数据长度是否至少为13字节 (1 header + 12 bytes for 6 s16 values)
    if #data < 13 then
        return
    end
    
    -- 检查数据是否以 MSG_HEADER(0xAA) 开头
    if string.byte(data, 1) == MSG_HEADER then
        -- 提取6个s16温度数据 (每个s16占2字节，共12字节)
        for i = 1, 6 do
            -- 计算字节位置 (第2-3字节为第一个s16，第4-5字节为第二个s16，以此类推)
            local byte1 = string.byte(data, i * 2)      -- 高字节
            local byte2 = string.byte(data, i * 2 + 1)  -- 低字节
            
            -- 组合成s16值 (小端序)
            local s16_value = byte2 * 256 + byte1
            
            -- 处理负值 (如果最高位为1，则是负数)
            if s16_value > 32767 then
                s16_value = s16_value - 65536
            end
            
            -- 除以100并保存到motor_temperature表
            motor_temperature[i] = s16_value / 100
        end

        if MOTOR_TEMP_LOG_EN:get() == 1 then
            -- 记录温度数据到日志 (使用浮点数格式，保留2位小数)
            logger:write('MTMP', 'T1,T2,T3,T4,T5,T6', 'ffffff',
                        motor_temperature[1], motor_temperature[2], motor_temperature[3],
                        motor_temperature[4], motor_temperature[5], motor_temperature[6])
        end


        
        -- 发送温度数据到Mission Planner的quick面板
        -- 使用named float发送温度数据，可以在MP中显示为temp1到temp6
        gcs:send_named_float("mot1_temp", motor_temperature[1])
        gcs:send_named_float("mot2_temp", motor_temperature[2])
        gcs:send_named_float("mot3_temp", motor_temperature[3])
        gcs:send_named_float("mot4_temp", motor_temperature[4])
        gcs:send_named_float("mot5_temp", motor_temperature[5])
        gcs:send_named_float("mot6_temp", motor_temperature[6])
        
        -- 记录接收到的温度数据
        if ENABLE_DEBUG then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Temp: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                motor_temperature[1], motor_temperature[2], motor_temperature[3],
                motor_temperature[4], motor_temperature[5], motor_temperature[6]))
        end
    end
end


-- 主更新函数
function update()
  -- 处理延迟逻辑
  if not delay_complete then
    delay_timer = delay_timer + UPDATE_RATE_MS / 1000 -- 转换为秒
    if delay_timer >= START_DELAY_SEC then
      delay_complete = true
    else
      -- 继续等待延迟完成
      return update, UPDATE_RATE_MS
    end
  end

  -- 初始化串口(如果尚未初始化)
  if not initialized then
    initialized = init_serial()
    if not initialized then
      -- 如果初始化失败，1秒后重试
      return update, UPDATE_RATE_MS
    end
  end



  -- 读取串口数据
  if initialized then
    if not port then
      gcs:send_text(MAV_SEVERITY.ERROR, "Port object missing")
      return update, UPDATE_RATE_MS
    end

    local nbytes = port:available()
    nbytes = 13
    if not nbytes then
      gcs:send_text(MAV_SEVERITY.ERROR, "available() returns nil")
      return update, UPDATE_RATE_MS
    end

    -- 确保有足够的字节来读取一个完整的数据包 (13字节)
    if nbytes >= 13 then
      -- 限制每次读取的字节数，优先处理完整的数据包
      nbytes = math.min(nbytes, MAX_BYTES_TO_READ)
      
      -- 读取数据
      local data = ""
      for i = 1, nbytes do
        local r = port:read()
        if r >= 0 then
          data = data .. string.char(r)
        end
      end

      -- 处理接收到的数据
      if #data >= 13 then
        process_received_data(data)
      end
    end
  end

  -- 继续循环
  return update, 50
end

-- 错误处理包装函数
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_SEVERITY.ERROR, "内部错误: " .. err)
    -- 发生错误时，1秒后重试，避免错误信息刷屏
    return protected_wrapper, 1000
  end
  return update()
end

-- 开始运行更新循环
return protected_wrapper()
