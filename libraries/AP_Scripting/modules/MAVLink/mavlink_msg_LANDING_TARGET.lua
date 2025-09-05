local LANDING_TARGET = {}
LANDING_TARGET.id = 149
LANDING_TARGET.fields = {
            {"time_usec","<I8"},  -- uint64_t
            {"target_num","<B"},  -- uint8_t
            {"frame","<B"},       -- uint8_t
            {"angle_x","<f"},     -- float
            {"angle_y","<f"},     -- float
            {"distance","<f"},    -- float
            {"size_x","<f"},      -- float
            {"size_y","<f"},      -- float
            {"x","<f"},           -- float
            {"y","<f"},           -- float
            {"z","<f"},           -- float
            {"q","<f",4},         -- float[4]
            {"type","<B"},        -- uint8_t
            {"position_valid","<B"},    -- uint8_t
             }
return LANDING_TARGET
