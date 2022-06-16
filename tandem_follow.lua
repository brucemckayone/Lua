-- TANDOM FOLLOW RECRUITMENT 
-- This set of functions implements a follow behaviour between two kilobots, a follower and a leader. 
    -- SCOUT
        -- The scout is given a noisy rando zig zag path
    -- FOLLOWER
        -- follows the random walker in a zig zag path
            

-- There has to be constant communication between follower and leader
-- the  behaviour is modeled after this video https://www.youtube.com/watch?v=Kam0jKiAk3c
    -- leader makes movement within the range of the followers ability to detect distance.
    -- follower makes searches a larger area than the leaders path whilst searching for the follower
    -- follower travels the orbit of leader, until distance is nill
    -- then starts on random path untill distance is again at its limits
    -- follower then continues along its previous path untill it reaches boundaries and follows orbit again 
        -- this produces a serpentine  or zigzag like search pattern for the follower.

-- HOMING BEHAVIOUR 
    -- when a food source is found, then this variaiable hasFoundFood set to true
        -- this then homes follower towards the colony(LIGHT SOURCE)
        
-- BUT HOW DO WE ESTABLISH A PATH BETWEEN SOURCE AND COLONY. 


-- CONSTANTS


local function set_motor(leftMotor,rightMotor)end

local function _delay_ms(delay)end

local function setDelayStartAndDoFunction(func)
    -- delay_start = sim.getSimulationTime()
    func()
end

local function asyncAwaitReturnSubstrate(substrate,delay,nextSubstrate)
    if(_delay_ms(delay)==1) then 
        substrate = nextSubstrate
    end
    return substrate
end

local dbug = {
    print_r = function(self,arr, indentLevel)
        local str = ""
        local indentStr = "#"
        if(indentLevel == nil) then
            print(self:print_r(arr, 0))
            return
        end
        for i = 0, indentLevel do
            indentStr = indentStr.."\t"
        end
        for index,value in pairs(arr) do
            if type(value) == "table" then
                str = str..indentStr..index..": \n"..print_r(value, (indentLevel + 1))
            else 
                str = str..indentStr..index..": "..value.."\n"
            end
        end
        return str
    end

}


local utils = {
    restrictNumberLimits =function(self,intensity,limit)
        if(intensity> limit)then intensity = limit end
        if(intensity< -limit)then intensity = -limit end
        return intensity
    end
}
local arrayUtils = {
   sum = function(array)
        local s = 0
        for i=1,#array do
            s=s+array[i]
        end
        return s
    end,
    average = function(self,array) 
        return self.sum(array)/#array
    end,

    diff = function(array)
        local derivitive ={}
        for i = 1,#array-1 do
            derivitive[i] = array[i+1] - array[i]
        end
        return derivitive
    end,

    roll = function (array, newValue,sampleSize)
        if(#array>= sampleSize)then
            table.remove(array,#array)
        end
        table.insert(array,1,newValue)
        return array
    end,
    linSpace = function(length)
        local r = {}
            for i = 1,length do
                table.insert(r,i);
            end
        return r;
    end

}

local Locomotion = {
        motorMaxIntensity =255,
        motorIntensity = 150,
        motorRatio =0.5,
        -- sets intensity of motor
        setMotorIntensity =  function(self,intensity)
            if(intensity> self.motorMaxIntensity)then intensity= self.motorMaxIntensity end
            self.motorIntensity = intensity
            self:setMotorRatio(self.motorRatio)
        end,

        -- set motor ratio used for turning
        setMotorRatio = function(self,ratio)
            --0.5 is straight lower is left higher is right
            self.motorRatio = ratio
            local leftMotorRatio = 1-self.motorRatio;
            set_motor(self.motorIntensity*leftMotorRatio,self.motorIntensity*self.motorRatio)
        end,

        zigZagMotorSubstrate = 0,
        doNoisyZigZagWalk = function(self)
            if(self.zigZagMotorSubstrate == 0 )then
                self.zigZagMotorSubstrate=1
                setDelayStartAndDoFunction(function()self:setMotorRatio(0.5)end) --go forward reset clock
            elseif (self.zigZagMotorSubstrate== 1) then
                self.zigZagMotorSubstrate=asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,6000,2)
            elseif (self.zigZagMotorSubstrate== 2) then
                self.zigZagMotorSubstrate = 3;
                setDelayStartAndDoFunction(function()self:setMotorRatio(0.9)end) -- go hard right restart clock
            elseif (self.zigZagMotorSubstrate==3) then
                self.zigZagMotorSubstrate=    asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,6000,4)
            elseif (self.zigZagMotorSubstrate==4) then
                self.zigZagMotorSubstrate= 5;
                setDelayStartAndDoFunction(function()self:setMotorRatio(0.1)end) --go left restart clock
            elseif(self.zigZagMotorSubstrate== 5)then
               self.zigZagMotorSubstrate= asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,6000,0)
            end
        end,

        distanceStraight1   = 0,
        distanceStraight2   = 0,
        distanceLeft        = 0,
        distanceRight       = 0,
        distanceHistoryStraight = {},
        distanceHistoryLeftTurn =  {},
        distanceHistoryRightTurn = {},
        

        calculateNeightbouringPostion = function(self) -- c = ?a² + b² - 2ab * cos( ? )
            local straight = arrayUtils:sum(arrayUtils:diff(self.distanceHistoryStraight))
            local toLeft   = arrayUtils:sum(arrayUtils:diff(self.distanceHistoryLeftTurn) )
            local toRight  = arrayUtils:sum(arrayUtils:diff(self.distanceHistoryRightTurn) )
             
            if( straight> 0)then
                print("is inf ront".. straight)
            else
                print("isBehind ".. straight)
            end
            
            if(toLeft> toRight)then
                print("is to left "..toLeft)
                else 
                print("is to right "..toRight)
            end
        end,

        distanceSubstrate   = 0,
        getNeighboursRelativePosition = function(self,distance) -- FUNCTION FOR FINDING STILL NEIGHTBOURS RELATIVE POSTION
            if(self.distanceSubstrate == 0 )then
                setDelayStartAndDoFunction(function()   -- ACTION 1 - COLLECT DATA -> GO STRAIGHT
                    self.distanceHistoryStraight = distance;  -- first distance data point saved at initial position
                    self.distanceSubstrate = 1            -- substrate set
                    self:setMotorRatio(0.5)             -- motor set straight
                end) --go forward reset clock
        
            elseif (self.distanceSubstrate== 1) then    -- CONTINUE FOR 200MS whilst going forward
                table.insert(self.distanceHistory, 1,distance)
                self.distanceSubstrate = asyncAwaitReturnSubstrate(
                    self.distanceSubstrate,200,2)

            elseif (self.distanceSubstrate== 2) then    -- ACTION 2 - COLLECT DATA -> GO RIGHT
                setDelayStartAndDoFunction(function()
                    self.distanceStraight2 = distance;  -- distance saved at second point.
                    self.distanceSubstrate = 3          -- substrateSet
                    self:setMotorRatio(1)             -- motor set right
                end) -- go hard right restart clock

            elseif (self.distanceSubstrate==3) then     -- CONTINUE FOR 200MS whilst turning right
                table.insert(self.distanceHistoryRightTurn, distance)
                self.distanceSubstrate = asyncAwaitReturnSubstrate(
                    self.distanceSubstrate,400,4)

            elseif (self.distanceSubstrate==4) then     -- ACTION 3 -   COLLECT DATA -> GO LEFT
                setDelayStartAndDoFunction(function()
                    self.distanceRight = distance        -- save distance from right turn
                    self.distanceSubstrate= 5           -- substrate set
                    self:setMotorRatio(0)             -- set motor left
                end) 
                --go left restart clock
            elseif(self.distanceSubstrate== 5)then      -- DELAY FOR 200 whilst turning left
                table.insert(self.distanceHistoryleftTurn, distance)
                self.distanceSubstrate= asyncAwaitReturnSubstrate(
                   self.distanceSubstrate,400,6)

            elseif (self.distanceSubstrate==6) then     -- ACTION 4 - STOP MOTOR CALCULATE RELATIVE POSITION
                setDelayStartAndDoFunction(function()
                    self:setMotorIntensity(0)           --  stop motors
                    self.distanceLeft = distance;    
                    self:calculateNeightbouringPostion()   --  save distance from left turn
                end)
            end
        end,



        spiralSubstrate = 0,
        doSpiralSearch = function(self)
            -- do an increasing spiral to find lost partner and reverse
            self.spiralSubstrate = self.spiralSubstrate + 1
            local spiralExpansionCount = 200
            self.setMotorRatio(self.spiralSubstrate/spiralExpansionCount)
            if(self.spiralSubstrate == spiralExpansionCount)then self.spiralSubstrate = 0 end
        end,

        leftMotorIntensity = 10,
        rightMotorIntensity = 10,
        setDirection = function(self,direction,intensity)
            local intensity = math.abs(intensity)
            if( direction) then
                self.leftMotorIntensity = utils:restrictNumberLimits( self.leftMotorIntensity*intensity, 255)
                self.rightMotorIntensity= utils:restrictNumberLimits(  intensity/self.rightMotorIntensity, 150)
            elseif (not direction) then
                self.rightMotorIntensity= utils:restrictNumberLimits(  intensity* self.rightMotorIntensity,255)
                self.leftMotorIntensity = utils:restrictNumberLimits( self.leftMotorIntensity/intensity,150)
            end
            set_motor(self.leftMotorIntensity,self.rightMotorIntensity)
        end,
        -------------------------------------------------
        sampleLength = 10,
        derHistory= {1,2,3,4},
        distanceHistory={0,0,0,0},
        sampleCount = 0,
        currentTurningDirection = false,
        minDistance  = function (self,distance)
                self.sampleCount = self.sampleCount +1
                self.distanceHistory = arrayUtils.roll(self.distanceHistory,distance, self.sampleLength)
                
                dbug:print_r(self.distanceHistory)
                local derHistorySample = arrayUtils.diff(self.distanceHistory)
            
                self.derHistory = arrayUtils.roll(self.derHistory, derHistorySample, self.sampleLength)
                -- if(#derHistorySample < self.sampleLength) then
                --     table.insert(self.derHistory,#self.derHistory+1, derHistorySample)
                -- else
                --     table.remove(self.derHistory,1);
                --     table.insert(self.derHistory,#self.derHistory, derHistorySample)
                -- end
                -- when a full sample is ganthered

                if(self.sampleCount==self.sampleLength)then 
                    self.sampleCount=0
                    
                    --determine path
                    local length = #self.derHistory
                    local x = {}
                    local y = arrayUtils.linSpace(length);

                    for i = 1,length do
                        local xi = arrayUtils.sum(self.derHistory[i])/length
                        table.insert(x,xi)
                    end
                    X = arrayUtils.sum(x)/#x
                    Y = arrayUtils.sum(y)/#y
                    local top = {}
                    local bottom = {}
                    
                    for i =1,length do
                        table.insert( top, (x[i]-X) * (y[i]-Y ) )
                        table.insert( bottom, ((x[i]-X))^2)
                    end
                    
                    self.rateOfRelativePostionChange = arrayUtils.sum(top)/arrayUtils.sum(bottom) --slope
                    self.rateOfRelativePostionChange = utils:restrictNumberLimits(self.rateOfRelativePostionChange,2)
                    
                    if( self.rateOfRelativePostionChange < 0 ) then
                        self:setDirection(not self.currentTurningDirection,self.rateOfRelativePostionChange)
                    else
                        self:setDirection(self.currentTurningDirection,self.rateOfRelativePostionChange)
                    end
                end
                
        end

    }

    for i=1,200 do
        local distance = math.floor(i % 4 * ( i % 2*i ) / 8)/1000
        Locomotion:minDistance(distance)
    end


    -- --random walk 
    -- local direction = 0
    -- local switchTime = 0

    -- if(direction== 0 )then
    --     local switchTime = math.random(0,50000)
    --     -- set_color(0,0,0) 
    --     -- delay_start = sim.getSimulationTime()
    --     local motorStrength = switchTime/50000
    --     direction = 1
    --     set_motor(200*motorStrength,200)
    -- elseif(direction==1) then
    --     if(_delay_ms(switchTime)==1)then direction = 2 end

    -- elseif(direction== 2)then
    --      switchTime = math.random(0,50000)
    --     -- delay_start = sim.getSimulationTime()
    --     local motorStrength = switchTime/50000
    --     set_motor(200,200*motorStrength)
    --     direction= 3
    -- elseif(direction== 3) then
    --     if(_delay_ms(switchTime)==1)then
    --     direction = 0
    --     end
    -- end