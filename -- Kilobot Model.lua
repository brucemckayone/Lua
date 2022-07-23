-- Kilobot Model
-- K-Team S.A.
-- 2013.06.24  
-- Adapted on 2021.11.16 by Coppelia Robotics
        
-- Add your own program in function user_prgm() , after the comment "user program code goes below"

function sysCall_init() 
        -- Check if we have a controller in the scene:
        i=0
        while true do
            h=sim.getObjects(i,sim.handle_all)
            if h==-1 then break end
            if sim.readCustomDataBlock(h,"kilobot")=='kilobotcontroller' then
                foundCtrller=true
                break
            end
            i=i+1
        end
        if not foundCtrller then
            sim.addLog(sim.verbosity_scripterrors,"The KiloBot could not find a controller. Make sure to have exactly one 'Kilobot_Controller' model in the scene.") 
        end
    
        -- save some handles
        KilobotHandle=sim.getObject('.') 
    
        LeftMotorHandle=sim.getObject('./revolute_jointLeftLeg')
        RightMotorHandle=sim.getObject('./revolute_jointRightLeg')
    
        MsgSensorsHandle=sim.getObject('./msgSensor')
    
        sensorHandle=sim.getObject("./proximity_sensor")
        BaseHandle=sim.getObject("./BatHold")
        
        obstacles=sim.createCollection(0)
        sim.addItemToCollection(obstacles,sim.handle_all,-1,0)
        sim.addItemToCollection(obstacles,sim.handle_tree,KilobotHandle,1)
    
    
        --BatGraphHandle=sim.getObject("BatGraph") -- should be uncommented only for one robot
    
        half_diam=0.0165 -- half_diameter of the robot base
        
        RATIO_MOTOR = 10/255 -- ratio for getting 100% = 1cm/s or 45deg/s
    
        -- 4 constants that are calibration values used with the motors
        cw_in_place     = 200 -- value for cw motor to turn the robot cw in place (note: ccw motor should be off)
        ccw_in_place    = 200 -- value for ccw motor to turn the robot ccw in place (note: cw motor should be off) 
        cw_in_straight  = 200 -- value for the cw motor to move the robot in the forward direction
        ccw_in_straight = 200 -- value for the ccw motor to move the robot in the forward direction 
    
    
        -- for battery management
        battery_init = 8000000  -- battery initial charged value
        battery =  battery_init    -- battery simulator (~5h with 2 motors and 3 colors at 1 level)
        factMoving = 100        -- factor for discharging during moving
        moving = 0                -- for battery discharging    1: one motor, 2: 2 motors, 0: not moving
        factCPU = 10            -- factor for discharging during cpu
        cpu=1                    -- cpu state: sleep = 0, active = 1
        factLighting = 40        -- factor for discharging during lighting
        lighting=0                -- for battery managing
        bat_charge_status=0        -- battery charge status 0= no, 1, yes    
        
        charge_rate=400         -- charge rate
        charge_max= battery_init-- battery_init*99.9/100 -- end of charge detection
    
        -------------------------------------------------------------------------------------------------------------------------------------------
        -- You should put your inital variable you would like to reset inside this function for your program.
        -- It is called when the reset button of the controller is pushed.
        reset_substate    = 0
    
        function run_reset()
            robot_id = math.random(0, 255) -- reset robot id
            -- below are variable to be reset for demo:
            substate1=0
            KiloState = 0
            KiloCounter = 0
            State = 0
            other_robots_id = {}
        end    
    
        
        -------------------------------------------------------------------------------------------------------------------------------------------
    
        -- for demo
        -- variables
        substate1=0
        KiloState = 0
        KiloCounter = 0
        State = 0
        other_robots_id = {}    
        -- constants
        SLAVE     = 1
        MASTER    = 2
        INIT    =    0
        WAIT    =    1
        PGMCODE    =    77
        
        
    
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
    end,
    printRawMessage= function(self)
        if(Com:isMessageInBuffer())then
            print("message_rx 1 =  ".. message_rx[1])
            print("message_rx 2 =  ".. message_rx[2])
            print("message_rx 3 =  ".. message_rx[3])
        end
    end,

}


local ArrayUtils = {
    tablelength = function (T)
        local count = 0
        for _ in pairs(T) do count = count + 1 end
        return count
    end,
    sum = function(self,array)
        local s = 0
        for i=1, #array do
                s = s + array[i]
        end
        return s
    end,
    average = function(self,array) 
        return self.sum(array)/#array
    end,
    averageBool =function(self,array)
        local falses = 0
        local trues = 0
        print(#array)
        for i=0,#array do
            if(array[i]==1)then
               falses= falses+ 1
            else
               trues = trues + 1
            end
        end
        if(falses> trues)then
            return false
        else
            return true
        end
        
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
    end,
    addOnlyUniqueElementsToArray = function(self,element,array) 
        array[element] = true
        return array
    end,

}

local Utils = {
    restrictNumberLimits =function(self,intensity,limit)
        if(intensity > limit)then intensity = limit end
        if(intensity < -limit)then intensity = -limit end
        return intensity
    end,
    getKeyFromValue= function (t, val)
        for k,v in pairs(t) do
            if(v == val)then 
            return k end
        end
    end,

    getDidget = function (self,num, digit)
       local s = string.sub(tostring(num), digit, digit)
       return tonumber(s)
    end,
    getDigitRange =function (self,num,start,stop)
       return string.sub(tostring(num), start, stop)
    end,
}
          
    local function setDelayStartAndDoFunction(func)
        delay_start = sim.getSimulationTime()
        func()
    end

    local function asyncAwaitReturnSubstrate(substrate,delay,nextSubstrate)
        if(_delay_ms(delay)==1) then 
            substrate = nextSubstrate
        end
        return substrate
    end

        
local Locomotion = {
        --------------------------------SET MOTOR RATIO -------------------------------
        -- set motor ratio used for turning--0.5 is straight lower is left higher is right
        --variables------------------------------------------------------------------------
            motorRatio =0.5,
            normalMotorIntensity = 150,
        -----------------------------------------------------------------------------------
            setMotorRatio = function(self,ratio)
               
                print(ratio)
                local leftMotorRatio = 1-ratio;
                set_motor(self.normalMotorIntensity*leftMotorRatio,self.normalMotorIntensity*ratio)
            end,
            
        -----------------------------------------STOP MOTOR --------------------------------
            stopMotor = function()set_motor(0,0)end,
        
        
    
        
        --------------------------------DO RANDOM ZIGZAG WALK---------------------------------------------------
        ---variables ---------------------------------------------------------------------------------------------------
            zigZagMotorSubstrate = 0,
        ---------------------------------------------------------------------------------------------------------------
            doNoisyZigZagWalk = function(self)
                if(self.zigZagMotorSubstrate == 0 )then
                    self.zigZagMotorSubstrate=1,
                    setDelayStartAndDoFunction(function()self:setMotorRatio(0.5)end) --go forward reset clock
                elseif (self.zigZagMotorSubstrate== 1) then
                    randomDelay =math.random(5000,10000)
                    self.zigZagMotorSubstrate=asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,2)
                elseif (self.zigZagMotorSubstrate== 2) then
                    self.zigZagMotorSubstrate = 3;
                    setDelayStartAndDoFunction(function()self:setMotorRatio(0.9)end) -- go hard right restart clock
                elseif (self.zigZagMotorSubstrate==3) then
                randomDelay =math.random(5000,10000)
                    self.zigZagMotorSubstrate=    asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,4)
                elseif (self.zigZagMotorSubstrate==4) then
                    self.zigZagMotorSubstrate= 5;
                    setDelayStartAndDoFunction(function()self:setMotorRatio(0.1)end) --go left restart clock
                elseif(self.zigZagMotorSubstrate== 5)then
                randomDelay =math.random(5000,10000)
                   self.zigZagMotorSubstrate= asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,0)
                end
            end,
            
        
        -----------------------SPIRAL SEARCHING-------------------------------------------------
        ---variables --------------------------------------------------------------------------
            spiralSubstrate = 0,
        ---------------------------------------------------------------------------------------
            doSpiralSearch = function(self)
                
                self.spiralSubstrate = self.spiralSubstrate + 1
                local spiralExpansionCount = 3000
                self:setMotorRatio(self.spiralSubstrate/spiralExpansionCount)
                if(self.spiralSubstrate == spiralExpansionCount)then self.spiralSubstrate = 0 end
            end,
            
            ---reset spiral
            resetSpiral = function(self)self.spiralSubstrate = 0 end,
       
        --------------------REDUCE DISTANCE BETWEEN SELF AND TARGET ----------------------
        ---variables--------------------------------------------------------------------------------- 
            sampleLength            = 5,
            derHistory              = {},
            distanceHistory         = {},
            sampleCount             = 0,
            currentTurningDirection = false,
        ---------------------------------------------------------------------------------------------
        
            minDistance  = function (self,distance)
                self.sampleCount = self.sampleCount +1  -- count used for conditional logic and timing
                -- creaters array  of length 30 that adds distance 
              
                table.insert(self.distanceHistory,distance)
                if(#self.distanceHistory == 50)then
                    local derHistorySample = ArrayUtils.diff(self.distanceHistory)
                    -- add derivitive to derHistory a list of arrays    
                    self.derHistory = ArrayUtils.roll(self.derHistory, derHistorySample, self.sampleLength)  -- 4
                    self.distanceHistory = {}
                end
               
                if(#self.derHistory==self.sampleLength)then   -- when a full sample is ganthered
                    
                    self.sampleCount= 0 -- reset count
                    
                    --determine path based on average of derivitives of der history
                    local length = ArrayUtils.tablelength(self.derHistory)         -- length for readablity
                    
                    local x = {}                            -- empty array for average function
                    local y = ArrayUtils.linSpace(length);  -- gradiated array count used as faux time steps
                    
                    -- loops over derivitive history and takes an average of each array in derHistory
                    -- adds outcome to xi
                    -- xi has list of either negative numbers or positive numbers of some magnitude 
                    for i = 1,length do 
                       
                        local xi = ArrayUtils:sum(self.derHistory[i])/length -- 
                        table.insert(x,xi)
                    end
                    
                    X = ArrayUtils:sum(x)/#x -- sim of all xi
                    Y = ArrayUtils:sum(y)/#y -- sum of all yi
                
                    
                    -- for readablity 
                    local top = {}
                    local bottom = {}
                    
                    -- for loop that calculates denominator and numorator arrays
                    -- used to calculate the slope of the derivitive arrays
                    for i =1,length do
                        table.insert( top, (x[i]-X) * (y[i]-Y ) )
                        table.insert( bottom, ((x[i]-X))^2)
                    end
                    -- sums top and bottom
                    local topSum = ArrayUtils:sum(top ) 
                    local bottomSum = ArrayUtils:sum(bottom)

                    -- calculates slope of array IE the rate of relative position change
                    -- if negative kilobot is moving away from at a rate = self.rateOfRelativePostionChange
                    self.rateOfRelativePostionChange = topSum /(bottomSum*1000) --slope
                    
                    -- function that limits the magnitude of self.rateOfRelativePostionChange
                    self.rateOfRelativePostionChange = Utils:restrictNumberLimits(self.rateOfRelativePostionChange,500)
                   -- print('rate of relative position change ' .. self.rateOfRelativePostionChange)
              
                    local direction = self.rateOfRelativePostionChange/math.abs(self.rateOfRelativePostionChange)
                    local intensity = math.abs(Utils:restrictNumberLimits( self.rateOfRelativePostionChange, self.motorMaxIntensity))
                    local ratio = ((self.motorMaxIntensity-intensity)/self.motorMaxIntensity)

                    -- print(intensity)
                    -- if(direction < 0)then
                    --     self.currentTurningDirection = not self.currentTurningDirection
                    -- end
                    
                    -- if(self.currentTurningDirection) then
                        
                        self.leftMotorIntensity = Utils:restrictNumberLimits( intensity, self.motorMaxIntensity)
                        self.rightMotorIntensity= Utils:restrictNumberLimits( intensity, 150)
                    -- else 
                    --     self.rightMotorIntensity= Utils:restrictNumberLimits(  intensity* self.rightMotorIntensity,self.motorMaxIntensity)
                    --     self.leftMotorIntensity = Utils:restrictNumberLimits( intensity/2,150)
                    -- end
                    set_motor(self.leftMotorIntensity,self.rightMotorIntensity)
                    self.derHistory={}
                    -- set
                    -- if( self.rateOfRelativePostionChange < 0 ) then -- if negative change position with intensity = self.rateOfRelativePostionChange
                    --     print("changing direction")
                    --     self:setDirection(  not self.currentTurningDirection,self.rateOfRelativePostionChange)
                    -- else -- remain on course and adjust intensity = self.rateOfRelativePostionChange
                    --     print('staying course')
                    --     self:setDirection( self.currentTurningDirection,self.rateOfRelativePostionChange)
                    -- end
                end
            end,
        ------------------------------  SET DIRECTION ---------------------------
        --variables ---------------------------------------------------------------
        leftMotorIntensity = 60,
        rightMotorIntensity = 60,
        motorMaxIntensity =255,
        ---------------------------------------------------------------------------
        setDirection = function(self,direction, intensity)
            local intensity = math.abs(intensity)
            print(intensity)
            if( direction) then
                self.leftMotorIntensity = Utils:restrictNumberLimits( intensity, self.motorMaxIntensity)
                self.rightMotorIntensity= Utils:restrictNumberLimits( intensity/2, 150)
            elseif (not direction) then
                self.rightMotorIntensity= Utils:restrictNumberLimits(  intensity* self.rightMotorIntensity,self.motorMaxIntensity)
                self.leftMotorIntensity = Utils:restrictNumberLimits( intensity/2,150)
            end
            set_motor(self.leftMotorIntensity,self.rightMotorIntensity)
        end,
        orbitSubstrate = 0, 
        setOrbitSubstrate = function(self,substrate)
             self.orbitSubstrate = substrate
             return self.orbitSubstrate
        end,
        orbit = function (self,inputDistance,minDistance,maxDistance)
            if (inputDistance ~= nil)  then 
                inputDistance = inputDistance + half_diam
               -- nameSuffix,kname=sim.getNameSuffix(sim.getObjectName(detectedObjectHandle))
                -- if the obstacle is another robot, light up the robot    
                if(inputDistance < minDistance) then                       --If distance is less than 4cm then
                        set_color(3,0,0)                            --Turn LED Red
                        set_motor(cw_in_straight,ccw_in_straight)   --Make the Kilobot move forwards
                 
                    elseif(inputDistance < maxDistance) then                   --If distance is between 4cm and 6cm then
                      set_color(0,3,0)                              --turn RGB LED Green         
                      set_motor(0,ccw_in_place)                     --Make the Kilobot turn left
                    elseif(inputDistance < 0.1) then                  --If distance is  6cm then                
                      set_color(0,0,3)                              --turn RGB LED Blue
                      set_motor(cw_in_place,0)                      --Make the Kilobot turn right        
                    else                   --If the Kilobot goes out of range then
                        set_color(3,0,3)                            --turn the LED purple and move straight 
                        set_motor(cw_in_straight,ccw_in_straight)
                    end    
            else
                set_color(3,0,3)                                    --If no criteria is met turn the LED purple and move straight
                set_motor(cw_in_straight,ccw_in_straight)
            end
        end,
        ------------------------------------ROTATIONAL SEARCH --------------------------------------------------------------------------
        rotationSearchArray ={0},
        directionCheck = {},
        isGoingTowards = false,
        isAtBoundary = false,
        hasRotationLockedOn=false,
        isSearchComplete=false,
        dataCollectionThreshold = 100,
        rotationSearchRoll = {},
        rotationSearchTickCount = 0,
        rotationSearch = function(self,distance)
            self.rotationSearchTickCount = self.rotationSearchTickCount+1
            -- self.rotationSearhRoll = ArrayUtils.roll(self.rotationSearchRoll,distance,400)
            -- print('rotation searchr roll ' .. ArrayUtils:sum(self.rotationSearhRoll))
            -- if(ArrayUtils:sum(self.rotationSearhRoll)==0)then
            --     self.rotationSearchTicks = 0
            --     self:doSpiralSearch()
            -- end


            if( distance ~= 0 or nil )then -- creating a distance history array used to check if moving towards or away from master point
                self.directionCheck = ArrayUtils.roll(self.directionCheck,distance,20)
                self.isGoingTowards = ArrayUtils:sum(ArrayUtils.diff(self.directionCheck)) > 0
            end

            if( (distance == 0 or nil)) then -- flag if outside boundary
                   -- print('no distance found')
                    self.isAtBoundary = true
                    self.dataCollectionThreshold = #self.rotationSearchArray
                else -- collect data 
                    --print('collecting data')
                    self.isAtBoundary = false
                    -- print('length of rotation search array = ' .. #self.rotationSearchArray )
                    --  print('distance is = '..distance)
                    table.insert(self.rotationSearchArray,distance)
                end
        
            if(self.hasRotationLockedOn)then -- if locked on to maximum go straight
                Light:setColor(Light.Colors.GREEN)
                --print(distance)
                
                set_motor(100,100) -- TODO: make this a zig zag forward
                if(#self.rotationSearchArray > 20)then -- if distance is increasing search again after 200 data points
                    print('greater than 20')
                    if(self.isGoingTowards~=true)then
                        print('switching to rotational search because current path has a positive gradient ie away from source')
                        self.rotationSearchArray = {}
                        self.hasRotationLockedOn=false
                    else
                        print('all good keep going')
                    end
                end


            
            elseif(self.hasRotationLockedOn == false and self.isAtBoundary == false and #self.rotationSearchArray < self.dataCollectionThreshold) then -- if not locked on collect data on rotation
                -- collecting data
                Light:setColor(Light.Colors.VIOLET)
                set_motor(100,0) -- start rotating
               
                -- print(#self.rotationSearchArray)
                -- print(Kilobot.tickCount)

                
                
                ZeroCheck = true
                --print(#self.rotationSearchArray >= self.dataCollectionThreshold)
            elseif(#self.rotationSearchArray >= self.dataCollectionThreshold or self.isAtBoundary == true  )then
                --print('Data Has Been Collected ')
                self.dataCollectionThreshold = 100
                set_motor(100,0)
                
                if(ZeroCheck)then 
                    ZeroCheck = false
                    for i =0,#self.rotationSearchArray do 
                        if(self.rotationSearchArray[i]==0)then table.remove(self.rotationSearchArray,i) end 
                    end

                end

                local maxDistance = math.max(table.unpack(self.rotationSearchArray))

                local minDistance = math.min(table.unpack(self.rotationSearchArray))
            
                local errorMargin  = 0.005
                local tangentToMasterPoint =  minDistance + (maxDistance/7)

                -- print('--------------------------------')
                -- -- print('max distance = ' .. maxDistance)
                -- print('min distance = ' .. minDistance)
                -- -- print('median distance  = ' .. medianDistance)
                -- print('targetDistance ' .. tangentToMasterPoint)
                -- print('current distance = ' .. distance)
                -- print('is kilobot traveling towards master = '.. tostring(self.isGoingTowards))
                
                if(#self.directionCheck == 20 and self.isGoingTowards )then
                    if( distance <= tangentToMasterPoint + errorMargin  and distance >= tangentToMasterPoint - errorMargin)then
                        --print('we are moving towards and taking aim')
                        if(  maxDistance ~= 0)then
                            --print('LOCKED ON setting path to straight line')
                            self.directionCheck={}
                            self.isSearchComplete=false
                            self.rotationSearchArray = {}
                            self.hasRotationLockedOn=true
                        end
                    end
                end
            end
        end,
    }
    
    Kilobot = { -- TAGKILOBOT
            
            tickCount = 0,
            --Kilobot Personal Id
            id = math.random(100,999), -- find bettwe way to generate unique id
            --current first order roll
            currentRoll     = 0, --tagcurrentroll
            -- list of first order rolls 
            Rolls= {
                FOLLOWER    = 0,
                LEADER      = 1,
                BEACON      = 2,
                FOODSOURCE  = 3,
                UNRECRUTED  = 4,
                SCOUT       = 5,
                BROOD       = 6,
                RECRUITED   = 7,
                },
            -- set the current user roll 
            setCurrentRoll = function(self, roll) 
                self.currentRoll = self.Rolls[roll]
            end,
            -- flag for has found food
            hasFoundFood = false,
            -- set flag for has found food
            setHasFoundFood = function(self,isFood)
                self.hasFoundFood = isFood
            end,
            isPlacedInHive = true,
            setIsPlacedInHive  = function(self,isPlaced)
                self.isPlacedInHive = isPlaced
            end,
        }
        
        Light = {

            Colors = {
                RED        = 0,
                GREEN      = 1,
                BLUE       = 2,
                ORANGE     = 3,
                PURPLE     = 4,
                YELLOW     = 5,
                WHITE      = 6,
                VIOLET     = 7,
                TURQUOISE  = 8,
            },
            
            setColor = function(self,color)
                if(color ==0)then
                    set_color(3,0,0) -- RED
                elseif(color== 1)then
                    set_color(0,3,0) -- GREEN
                elseif(color== 2)then
                    set_color(0,0,3) -- BLUE
                elseif(color== 3)then
                    set_color(3,3,0) -- ORANGE
                elseif(color== 4)then
                    set_color(0,3,3) -- PURPLE
                elseif(color== 5)then
                    set_color(3,1,0) -- YELLOW
                elseif(color== 6)then
                    set_color(3,3,3) -- WHITE
                elseif(color== 7)then
                    set_color(3,0,3) -- VIOLET
                elseif(color== 8)then
                    set_color(0,3,1) -- TURQUOISE
                end
            end,
            
            gradientDistance = function(self,distance) 
                if (distance ~= nil)  then 
                    distance= distance + half_diam
                    if sim.readCustomDataBlock(detectedObjectHandle,"kilobot")=="detectable" then
                        if(distance < 0.033) then
                            self:setColor(self.Colors.WHITE) --turn RGB LED White
                        elseif(distance < 0.040) then
                            self:setColor(self.Colors.RED)
                        elseif(distance < 0.050) then
                            self:setColor(self.Colors.ORANGE)
                        elseif(distance < 0.060) then
                            self:setColor(self.Colors.GREEN)
                        elseif(distance < 0.070) then            
                            self:setColor(self.Colors.TURQUOISE)
                        elseif(distance < 0.080) then
                            self:setColor(self.Colors.BLUE)
                        elseif(distance < 0.090) then
                            self:setColor(self.Colors.VIOLET)                
                        else
                            set_color(0,0,0)     
                        end     
                    end
                else
                    set_color(0,0,0)
                end
            end
        }
        
        Com = { -- messages are sent every duty cycle  --TAGCOM

            searchId = 100,

            functions = {
                DISTANCE            = 0,
                NEIGHBOURS          = 1,
                MIN_NEIGHBOURS      = 2,
                RECRUIT             = 3,
                SEND_RECRUITABLE_ID = 4,
            },

            getMessage = function(self) 
                get_message()
            end,
            setMessage = function(self,bit1,bit2,bit3) 
                if(bit1==nil) then bit1 =0 end
                if(bit2==nil) then bit2 =0 end
                if(bit3==nil) then bit3 =0 end
                message_out(bit1,bit2,bit3)
            end,
            messageDelayCount = 0,
            setMessageWithDelay =  function(self,bit1,bit2,bit3,delay)
                self.messageDelayCount = self.messageDelayCount + 1
                if(self.messageDelayCount > delay)then self.messageDelaycount = 0 end -- reset if over mabye not needed 
                if(self.messageDelayCount == delay)then
                    message_out(bit1,bit2,bit3)
                    self.messageDelayCount=0
                end
            end,
            resetMessage =function(self)
                message_rx[1] = 0
                message_rx[2] = 0
                message_rx[3] = 0
            end,
            isOfComType = function (self,comType)
              if(comType == Utils:getDidget(message_rx[1],5) and comType ~= nil) then return true else return false end
            end,
            
            -- this function will reset messages if buffer has been ampty of new messages after 100 ticks 
            tickCount =0,
            sanitizeBuffer = function(self)
                if(self:isMessageInBuffer())then
                    --print(message_rx[1])
                    self.tickCount= 0
                else
                    self.tickCount =self.tickCount+1
                    if(self.tickCount == 40) then 
                        self.tickCount = 0
                         message_rx[1] = 0
                         message_rx[2] = 0
                         message_rx[3] = 0
                    end
                    
                end
            end,
           
            isMessageInBuffer = function(self,action) 
                self:getMessage()
                if(message_rx[6]==1)then
                    if(action~= nill) then action()end
                    return true
                else
                    return false
                end 
            end,
            hasMessage = function(self)
                if(message_rx[6]==1)then return true else return false end
            end,
            hasMessageOfType = function(self,type)
                if(self:hasMessage())then
                    if(self:isOfComType(type))then
                        return true
                    else
                        return false
                    end
                end
            end,
            messageIsFromRoll = function(self,roll)
                if(Utils:getDidget(message_rx[1],1) == roll) then
                    return true
                else
                    return false
                end
            end,
            getMessageAsColor = function(self)
               self:isMessageInBuffer(function()
                print(Utils.getKeyFromValue(Light.Colors,message_rx[1]))
               end )
            end,
            
            
            printRawMessage= function(self)
                if(self:hasMessage())then
                    print("message_rx 1 =  ".. message_rx[1])
                    print("message_rx 2 =  ".. message_rx[2])
                    print("message_rx 3 =  ".. message_rx[3])
                end
            end,
            getMessageAsRoll = function(self)
             self:isMessageInBuffer(function()
                print(Utils.getKeyFromValue(Kilobot.Rolls,message_rx[1]))
               end )
            end,
            
            
            falses =0,
            trues = 0,
            isTalkingToBeacon = function(self)
                if(message_rx[6]==1)then
                   if(message_rx[1]==Kilobot.Rolls.BEACON)then
                        self.falses = self.falses +1
                   else
                        self.trues = self.trues +1
                   end
                   if(self.trues >self.falses)then return true else return false end
                   if(self.trues> 50 or self.falses> 50) then self.trues=0;self.falses=0 end
                end
            end,

            decodeUniqueId = function (self, encodedData) -- gets last unique Id From Recruits from first bit ignore first 
                return tonumber(Utils:getDigitRange(encodedData,2,4))
            end,
            
            decodeUserRoll = function (self,encodedData) 
                return Utils:getDidget(encodedData,1)
            end,

            -- communications list refreshCount
            refreshCount = 0,
            -- list of ids in which current Kilobot is in communications with
            inCommunicationIdsSet    = {},
            currentMinNeighbours     = 99,
            myNeighboursCounts       = 0,
            messageSwitchSubstrate   = 0,
            listOfEdgeIds            = {},
            isRecruitable            = false,
            refreshRate              = 200,

            isOnEdgeOfHive = function(self)
                self.refreshCount =self.refreshCount + 1

                if(self.refreshCount < 100)then
                    --MESSAGE SET
                    self:setMessage(Kilobot.currentRoll .. Kilobot.id .. self.functions.MIN_NEIGHBOURS, self.currentMinNeighbours,0)
                    if(self:isOfComType(self.functions.MIN_NEIGHBOURS))then
                        if(message_rx[2] < self.currentMinNeighbours)then
                            self.currentMinNeighbours = message_rx[2]
                        elseif(self.myNeighboursCounts ~= 99 and self.myNeighboursCounts < self.currentMinNeighbours)then 
                            self.currentMinNeighbours = self.myNeighboursCounts
                        end
                        
                    end
                --establish unique neighbours of self
                elseif (self.refreshCount > 100 and self.refreshCount < 150) then
                    if(self:hasMessage())then -- if message from recruit add id to list 
                        if(self:messageIsFromRoll(Kilobot.Rolls.UNRECRUTED)) then
                            self.inCommunicationIdsSet[self:decodeUniqueId(message_rx[1])] = true -- adds only unique
                            if(self:isOfComType(self.functions.NEIGHBOURS))then
                                if(self.inCommunicationIdsSet ~=nil) then -- tell how many NEIGHBOURS you have
                                    self.myNeighboursCounts = ArrayUtils.tablelength(self.inCommunicationIdsSet)
                                end
                            end 
                        end
                    end
                    --MESSAGE SET
                    self:setMessage(Kilobot.Rolls.UNRECRUTED .. Kilobot.id .. self.functions.NEIGHBOURS, ArrayUtils.tablelength(self.inCommunicationIdsSet),0 )
                        
                     elseif( self.refreshCount == self.refreshRate )then
                    print('---------------')
                    print(Kilobot.id .. ' is in communication with ' ..  ArrayUtils.tablelength(self.inCommunicationIdsSet) .. ' kilobots')
                    print('currentMinValue ' ..  self.currentMinNeighbours .. ' kilobots')
                    self.listOfEdgeIds ={}
                    self.currentMinNeighbours = 99
                    self.refreshCount = 0
                    self.inCommunicationIdsSet = {}
                

                if(self.currentMinNeighbours ==self.myNeighboursCounts) then
                     self.isRecruitable = true
                     return true
                else
                    self.isRecruitable =false
                    return false
                end
                    end
                -- THE GREAT REFRESH
                
           
            end,

            sendEdgeId = function(self,id)
                --print(Kilobot.Rolls.BROOD .. Kilobot.id .. self.functions.SEND_RECRUITABLE_ID)
                self.setMessage(Kilobot.Rolls.UNRECRUTED .. Kilobot.id .. self.functions.SEND_RECRUITABLE_ID,0,0)
            end,
            
            sendRecruitmentMessage = function(self,recruitementInfo)
                self:setMessage(Kilobot.Rolls.BROOD .. Kilobot.id .. self.functions.RECRUIT,0 )
            end,
        }
    
        
        Behaviours = { --tagbehaviour
            go = true,
            leader= function(self,distance)
               
                if (distance ~= nil)  then
                    -- print('LeaderDistance = ' .. distance) 
                    --distance= distance + half_diam
                    --if sim.readCustomDataBlock(detectedObjectHandle,"kilobot")=="detectable" then
                        if(distance > 0.07) then self.go = false end
                        if(distance < 0.03) then self.go = true end
                        if(self.go) then
                           -- print('leading is doing search and distance = '..  distance)
                            Locomotion:doNoisyZigZagWalk()
                        else 
                             -- print('leading is stopped = '..  distance)
                            Locomotion:stopMotor()
                        end 
                    --end
                end
            end,
            
            scout = function(self)
                Locomotion:doNoisyZigZagWalk()
                Light:setColor(Light.Colors.RED)
            end,
            
            follower = function(self,distance)
                local isSearching = false
                local initSubstrate = 0
                --print(distance)
                if (distance ~= nil)  then
                    -- distance = distance + half_diam
  --                      if sim.readCustomDataBlock(detectedObjectHandle,"kilobot")=="detectable" then
                            --print('doing follow')
                        if (distance < 0.03) then self.go = false end
                        if ( distance > 0.06  ) then self.go = true end
                        
                        if(self.go )then 
                            --print('Follower doing Rotation Search with distance = ' .. distance )
                            --  print(distance)
                            Locomotion:rotationSearch(distance)
                            set_color(0,5,0)
                        else
                            Locomotion.rotationSearchArray ={0}
                            Locomotion.directionCheck = {}
                            Locomotion.isGoingTowards = false
                            Locomotion.isAtBoundary = false
                            Locomotion.hasRotationLockedOn=false
                            Locomotion.isSearchComplete=false
                            Locomotion.dataCollectionThreshold = 100
                            Locomotion.rotationSearchRoll = {}
                            Locomotion.rotationSearchTickCount = 0
                            -- print('follower stoped with  distance = ' .. distance)
                            Locomotion:stopMotor()
                            set_color(0,0,5)
                        --  end
                    end
                -- else
                --     print('follower doing spiral')
                --     Light.setColor(Light.Colors.RED)
                --     Locomotion:stopMotor()
                --     --Locomotion:resetSpiral()
                --     Locomotion:doSpiralSearch()
                end
            end,
            
            foodSource= function()
                --transmit distance two someone who finds a beacon
                --byte 1 defines roll byte 2 defines distance 
                local result,distance,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.checkProximitySensor(sensorHandle,obstacles)
                -- sending out distance ^6 to maintain fedelity. 
                if(distance==nil) then distance =0 end 
                Com:setMessage(2,distance*10^6,0)
                --Com:printRawMessage()
            end
            
        }
        
    
        -------------------------------------------------------------------------------------------------------------------------------------------
        -- Functions similar to C API
        -------------------------------------------------------------------------------------------------------------------------------------------    
        --MAINLOOP-----------------------------------------------------------------------------------------------------------------------------------------
        function user_prgm()
            Com:sanitizeBuffer()
            local result,distance,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.checkProximitySensor(sensorHandle,obstacles)
            
            if(Kilobot.currentRoll == Kilobot.Rolls.FOLLOWER)then -- TAGFOLLOWER

                -- minimize distance between follower and leader, 
                -- if distance is above X
                    -- start minimizing
                -- if distance is below Y
                    -- stop and wait for distance x again. 
                
                -- if message_rx 2 =  'found food source'
                    -- start widest orbit of leader
                -- if find foodsource
                    -- set currentRoll to recruited with given params

                --Com:printRawMessage()
                Behaviours:follower(distance)
                -- if(message_rx[1]==Kilobot.Rolls.LEADER)then
                --     Behaviours:follower(message_rx[2]/10^6)       
                -- end
                -- if(distance == nil) then distance = 0 end
                -- Com:setMessage(Kilobot.currentRoll,distance*10^6,0)
                
                
            elseif (Kilobot.currentRoll == Kilobot.Rolls.LEADER) then --TAGLEADER
                
                --print(distance)
                
                Behaviours:leader(distance)
                
           
                
                
            elseif (Kilobot.currentRoll == Kilobot.Rolls.BEACON) then --TAGBEACON
                -- Do beacon behaviour
                
            elseif (Kilobot.currentRoll == Kilobot.Rolls.FOODSOURCE)  then --FOODTAG
                -- Do food source behaviour
                    -- default transmit message of I AM FOOD
                    -- and distance is reduced to threshold transfer data to finder 
                    -- if found 
                        -- set flag as found food source 
                    -- if set to found 
                        --transmit data to finder 
                            -- amount of food left
                            -- amount of food given
                            -- food quality 
                            -- etc
                    Behaviours:foodSource()
                
            elseif (Kilobot.currentRoll == Kilobot.Rolls.UNRECRUTED )  then --TAGUNRECRUITED
                
                --Locomotion:orbit(distance,0.03,0.05)
                --print()
                

                
                if(Com:isOnEdgeOfHive()) then
                        Light:setColor(Light.Colors.GREEN)
                        --print('green')
                        if (Com.refreshCount > 150 and Com.refreshCount < Com.refreshRate) then
                            print('bot id = ' .. Kilobot.id ..' has Edge Position')
                            Com:setMessage(Kilobot.currentRoll ..Kilobot.id .. Com.functions.SEND_RECRUITABLE_ID, Kilobot.id,0)
                            
                        end
                else
                        --print('yellow')
                        Light:setColor(Light.Colors.YELLOW)
                         if (Com.refreshCount > 150 and Com.refreshCount < Com.refreshRate) then
                            print('--------------------- its on ==============')
                            if(Com.hasMessage())then
                            --print('yellow message')
                            -- Com:printRawMessage()
                            -- print('yellow message printed')
                            if(Utils:getDidget(message_rx[1],5)==Com.functions.SEND_RECRUITABLE_ID)then
                                -- print('has correct message')
                                print('bot id = ' .. message_rx[2] .. ' has Edge Position detected from non edge')
                                --self.listOfEdgeIds[self:decodeUniqueId(message_rx[1])]=true
                                -- self:sendEdgeId(self:decodeUniqueId(message_rx[1]))
                                -- print(self:decodeUniqueId(message_rx[1]))
                                -- print(self.listOfEdgeIds)
                               Com:setMessage(Kilobot.currentRoll .. message_rx[2] .. Com.functions.SEND_RECRUITABLE_ID, message_rx[2],0)
                                --self:printRawMessage() 
                            end
                        end
                        end
                end
                -- if (Com.refreshCount > 150 and Com.refreshCount < Com.refreshRate) then
                --     if(Com.isRecruitable==true) then -- random value for each kilobot to 
                --             print('bot id = ' .. Kilobot.id ..' has Edge Position')
                --             Com:setMessage(Kilobot.currentRoll ..Kilobot.id .. Com.functions.SEND_RECRUITABLE_ID, Kilobot.id,0)
                --             --self.isRecruitable = true
                --     elseif(Com.isRecruitable == false)then --listen for message and transmit
                --         if(Com.hasMessage())then
                --             print('yellow message')
                --             Com:printRawMessage()
                --             print('yellow message printed')
                --             if(Utils:getDidget(message_rx[1],5)==Com.functions.SEND_RECRUITABLE_ID)then
                --                 print('has correct message')
                --                 print('bot id = ' .. message_rx[2] .. ' has Edge Position detected from non edge')
                --                 --self.listOfEdgeIds[self:decodeUniqueId(message_rx[1])]=true
                --                 -- self:sendEdgeId(self:decodeUniqueId(message_rx[1]))
                --                 -- print(self:decodeUniqueId(message_rx[1]))
                --                 -- print(self.listOfEdgeIds)
                --                Com:setMessage(Kilobot.currentRoll .. message_rx[2] .. Com.functions.SEND_RECRUITABLE_ID, message_rx[2],0)
                --                 --self:printRawMessage() 
                --             end
                --         end
                --     end
                -- end
                -- print(tx0 .. ' ' ..tx1..' ' ..tx2)


                -- if(Com.refreshCount < 45)then
                --     -- print('-------')
                --     -- print('sending min')
                --     -- Com:printRawMessage()
                    
                --     Com:setMessage(Kilobot.currentRoll .. Kilobot.id .. Com.functions.MIN_NEIGHBOURS, Com.currentMinNeighbours,0)
                --     if(Com:isOfComType(Com.functions.MIN_NEIGHBOURS))then
                --         -- print(message_rx[2])
                --         -- print(Com.currentMinNeighbours)
                --         if(message_rx[2] < Com.currentMinNeighbours)then
                --             Com.currentMinNeighbours = message_rx[2]

                --             --print('min neighbours message' ..Com.currentMinNeighbours)
                --         elseif(Com.myNeighboursCounts ~= 99 and Com.myNeighboursCounts < Com.currentMinNeighbours)then 
                --             Com.currentMinNeighbours = Com.myNeighboursCounts
                --             --print('min neighbours message' .. Com.currentMinNeighbours)
                --         end
                --     end
                -- elseif (Com.refreshCount > 45 and Com.refreshCount < 100) then

                --     -- print('-------')
                --     -- print('sending normal')
                --     -- Com:printRawMessage()

                --     if(Com:hasMessage())then -- if message from recruit add id to list 
                --         -- print('sentry in commuication with roll = '.. Utils:getDidget(message_rx[1],1))
                --         if(Com:messageIsFromRoll(Kilobot.Rolls.UNRECRUTED)) then
                --         --print('is recruited merssage ')
                --             --print('sentry is in communication with unrecruited with unique = ' .. Com:decodeUniqueId(message_rx[1]))
                --             Com.inCommunicationIdsSet[Com:decodeUniqueId(message_rx[1])] = true -- adds only unique
                --             if(Com:isOfComType(Com.functions.NEIGHBOURS))then
                --                 -- print('got message')
                --                 if(Com.inCommunicationIdsSet ~=nil) then -- tell how many NEIGHBOURS you have
                --                     Com.myNeighboursCounts= ArrayUtils.tablelength(Com.inCommunicationIdsSet)
                --                     --if(Com.currentMinNeighbours == 99) then Com.currentMinNeighbours = Com.myNeighboursCounts end
                --                 end
                --             end 
                --         end
                --     end
                --     Com:setMessage(Kilobot.Rolls.UNRECRUTED .. Kilobot.id .. Com.functions.NEIGHBOURS, ArrayUtils.tablelength(Com.inCommunicationIdsSet),0 )
                -- end
               
                -- --Com:printRawMessage()

                
                -- Com.refreshCount =Com.refreshCount +1
                -- if( Com.refreshCount == 100 )then
                    
                --     print('---------------')
                --     print(Kilobot.id .. ' is in communication with ' ..  ArrayUtils.tablelength(Com.inCommunicationIdsSet) .. ' kilobots')
                --     print('currentMinValue ' ..  Com.currentMinNeighbours .. ' kilobots')
                --     Com.currentMinNeighbours = 99
                --     Com.refreshCount = 0
                --     Com.inCommunicationIdsSet = {}
                -- end
                -- if has distance but no com rotational search if has com orbit com with distance untill encoutering correct id then stop 
                -- if(distance==nil)then distance= 0 end
                -- if(distance > 0.06)then Locomotion.orbitSubstrate = 0 end
                
                -- Locomotion.orbitSubstrate = asyncAwaitReturnSubstrate(Locomotion.orbitSubstrate,20000,1)

                -- if(Locomotion.orbitSubstrate == 0)then
                --     Light:setColor(Light.Colors.PURPLE)
                --     print('in Orbit')
                --     Locomotion:orbit(distance,0.06,0.07)
                -- elseif(Locomotion.orbitSubstrate==1)then
                --    --print('inSpiralSearch')
                --     if(Kilobot.isPlacedInHive) then
                --         print('I AM IN PLACE id = ' .. Kilobot.id)
                --         Locomotion:stopMotor()
                --         if(Com:hasMessageOfType(Com.functions.NEIGHBOURS))then -- TODO: this should be communicated less often
                --             --update search ids
                --             Com.searchId = Utils.getDidget(message_rx[2],1)
                --             Com:setMessageWithDelay(Kilobot.currentRoll .. Kilobot.id ..  Kilobot.functions.NEIGHBOURS,100)
                --         end
                --     else -- if in communication with hive ie NEIGHBOURS
                --             -- if transmitted Id is same as neighbours stop
                --             -- esle min distance then Orbit

                --         if(Com:hasMessageOfType(Com.functions.DISTANCE))then
                --             if(Com:decodeUniqueId(message_rx[1]) == 100) then -- incommunication with seed
                --                      -- local distance = message_rx[2]/10^6
                --                      print('spiral 1')
                --                      print(distance > 0.050)
                --                      print(distance)
                        
                --                     Locomotion:rotationSearch(distance)
                --                      --print(distance > 0.6)
                --                     if( distance < .02 )then
                --                         -- Locomotion.orbitSubstrate
                --                         Kilobot:setIsPlacedInHive(true)
                --                     elseif(distance > 0.05)then
                --                         Locomotion:stopMotor()
                --                         print('setting orbit substrate')
                --                         Locomotion:setOrbitSubstrate(0)
                --                     end
                --             else
                --                 if(Utils:getDidget(message_rx[2],1)==Com.searchId)then
                --                     print('spiralcom')
                --                     local distance = Utils:getDigitRange(message_rx[2]/10^6,2,7)
                --                     Locomotion:rotationSearch(distance)
                --                 end
                --             end
                --         else
                --             Locomotion:rotationSearch(distance)
                --         end
                --     end
                -- end     -- if(Com:hasMessage())then
                --     -- print('unrecruited has Message = ' .. message_rx[1])
                --     if(Com:decodeUserRoll()==Kilobot.Rolls.BROOD) then 
                --         --set coordinate from sentry based on message_rx[2]
                --         print('number of close ids =' .. message_rx[2])
                --     end
                --     if(Com:decodeUserRoll()==Kilobot.Rolls.UNRECRUTED)then 
                --         if(message_rx[2]== coordinateFromSentry - 1)then
                --              Locomotion.stopMotor()
                --         else
                --             if(distance < 0.4) then
                --                 Locomotion:orbit(distance,0.4)
                --             else
                --                 Locomotion:minDistance(distance)
                --             end
                --         end
                --     end
                -- end

                -- cluster around sentry establish distance hierarche
                    --settle when all are in correct place
                    --wait for instruction
                    
                    --if instruction to recruit
                        -- establish route ID,
                        -- establish personal ID,
                        --setCurrentRollToRecruited

            elseif (Kilobot.currentRoll == Kilobot.Rolls.RECRUITED )  then --TAGRECRECRUITED
                -- follow beacon 
                -- if interacting with food source
                    -- change light to green
                    -- gather foodsource data
                -- if interacting with correct sentry
                    -- change light to off
                    -- transfer data to sentry
                -- continue
                
            elseif (Kilobot.currentRoll == Kilobot.Rolls.SCOUT )  then --TAGSCOUT
            -- Do food source behaviour
            -- random walk search unless near beacon if near beacon follow beacon
             if(message_rx[1]==Kilobot.Rolls.FOODSOURCE)then
                -- found FOODSOURCE
                Behaviours:follower(message_rx[2]/(10^6))
                --if distance is of threshold 
                    --set currentRoll To Recruit
            else
                Behaviours:scout()
            end

            elseif (Kilobot.currentRoll == Kilobot.Rolls.BROOD) then --TAGBROOD

                Com.refreshCount=Com.refreshCount+1
                if(Com.refreshCount > 150 and Com.refreshCount<Com.refreshRate)then
                    if(Com:hasMessage())then
                        print(message_rx[1])
                        Com:printRawMessage()
                        -- print('------------')
                        -- print('has Message from ' .. Utils:getDidget(message_rx[1],5) )
                            if(Utils:getDidget(message_rx[1],5) ==Com.functions.SEND_RECRUITABLE_ID)then
                                print('has correct message')
                                Com:printRawMessage()
                                Com.listOfEdgeIds[Com:decodeUniqueId(message_rx[1])]=true
                                Com:sendEdgeId(Com:decodeUniqueId(message_rx[1]))
                                print(Com.listOfEdgeIds)
                            end
                    end
                else
                    Com.listOfEdgeIds={}
                    Com.refreshCount=0
                end
            
            -- get list of edge ids in a group
                


            --     Light:setColor(Light.Colors.BLUE)
            --     -- do sentry behaviour
            --     -- constantly transmit number of neighbours 
            --         -- this will be echoed through out the hive,
            --             -- will be used to give stopDistance
            --     Com.refreshCount = Com.refreshCount + 1
            --     Com:setMessage( Kilobot.Rolls.BROOD .. Kilobot.id  .. Com.functions.DISTANCE, distance * 10^6, 0) -- set message comminicating (ROLL,number of unrecruited, 0)
            --     if(Com:hasMessage())then -- if has message
            --         -- print('sentry in commuication with roll = '.. Utils:getDidget(message_rx[1],1))
            --         if(Utils:getDidget(message_rx[1],1) == Kilobot.Rolls.UNRECRUTED) then
            --             --print('sentry is in communication with unrecruited with unique =' .. Com:decodeUniqueId(message_rx[1])
            --             Com.inCommunicationIdsSet[Com:decodeUniqueId(message_rx[1])] = true -- adds only unique
            --             if(Com.inCommunicationIdsSet ~=nil)then
            --                Com:setMessage(Kilobot.Rolls.BROOD .. Kilobot.id .. Com.functions.NEIGHBOURS, #Com.inCommunicationIdsSet )
            --             end
            --         end
            --     end

            --     if( Com.refreshCount == 100 )then
            --         Com.refreshCount = 0
            --         Com.inCommunicationIdsSet = {}
            --     end
            end
        end
        
        -------------------------------------------------------------------------------------------------------------------------------------------
        
        -- Set motor speed PWM values for motors between 0 (off) and 255 (full on, ~ 1cm/s) for cw_motor and ccw_motor 
        function set_motor (cw_motor,ccw_motor)
        -- Set speed
            sim.setJointTargetVelocity(RightMotorHandle,ccw_motor*RATIO_MOTOR)
            sim.setJointTargetVelocity(LeftMotorHandle,cw_motor*RATIO_MOTOR)
    
            -- for battery managing
            if ((cw_motor == 0) and (ccw_motor==0)) then
                moving=0
            elseif ((cw_motor == 0) or (ccw_motor==0)) then
              moving=1    
            else
            -- both moving
              moving=2
            end
    
        end
    
        -- Set LED color, values can be from 0(off)-3(brightest) 
        function set_color(r,g,b)
            sim.setShapeColor(BaseHandle,"BODY",0,{r*0.6/3.0+0.3, g*0.6/3.0+0.3, b*0.6/3.0+0.3})
            lighting=r+g+b
        end
    
        -- Print integer over StatusbarMessage (serial port for real robot) - be careful can effect timing! 
        function kprinti(int)
            sim.addLog(sim.verbosity_msgs,int)
        end
    
        -- Print string up to 10 characters - be careful, can effect timing! 
        function  kprints(string)
            sim.addLog(sim.verbosity_msgs,string)
        end
    
        -- Set message values to be sent over IR, 3 bytes
        -- tx0,tx1,tx2 (tx2 lsb must not be used)
        tx0=0
        tx1=0
        tx2=0
        function message_out(_tx0,_tx1,_tx2)
          tx0=_tx0
          tx1=_tx1
          tx2=_tx2    
        end
    
        -- received message
        message_rx = {0,0,0,0,0,0};
        enable_tx = 0 -- to turn on/off the transmitter
        senderID = nil
    
        -- Take oldest message off of rx buffer message. It is only new if message_rx[5]==1
        -- If so, message is in message_rx[0] and message_rx[1] 
        -- distance to transmitting robot (in mm) is in message_rx[3].
        function get_message()        
            if (data ~= nil) then
                udata=sim.unpackInt32Table(data)
                message_rx[1]=udata[1]
                message_rx[2]=udata[2]
                message_rx[3]=udata[3]
                
                result,distance,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.checkProximitySensor(sensorHandle,senderHandle)
                
                if (result == 1) then
                    
                    message_rx[4]=(distance+half_diam)*1000  -- distance in mm + 1/2diameter of robot
    
                    message_rx[6]=1    -- message receveid
    
                else
                    message_rx[6]=0 -- no message receveid
                end            
    
            else
                message_rx[6]=0 -- no message receveid
            end
    
        end
        
        special_mode = 1
        run_program = 0
        special_mode_message = 3
    
        function receive_data()
            -- receive latest message and process it
            
            data,senderID,dataHeader,dataName=sim.receiveData(0,"Message",MsgSensorsHandle)
            
            if (data ~= nil) then
                senderHandle= sim.getObjectAssociatedWithScript(senderID)
                udata=sim.unpackInt32Table(data)
                message_rx[1]=udata[1]
                message_rx[2]=udata[2]
                message_rx[3]=udata[3]
    
                -- special message
                if (message_rx[3] == 0x1) then
                    special_mode_message=message_rx[2]
                    special_mode = 1
    
                end
            end
        end
    
        irstart=sim.getSimulationTime()
    
        function send_data() -- send data from ir every 0.2s, at a max distance of 7cm
            newir=sim.getSimulationTime()
            if ((enable_tx==1) and (newir-irstart>0.2)) then
                sim.sendData(sim.handle_all,0,"Message",sim.packInt32Table({tx0,tx1,tx2}),MsgSensorsHandle,0.07,3.1415,3.1415*2,0.8)
                irstart=newir
            end 
            
        end
    
        -- Measure battery voltage, returns voltage in .01 volt units
        -- for example if 394 is returned, then the voltage is 3.94 volts 
        function measure_voltage()
            return battery*420/battery_init
        end
    
        -- Measure if battery is charging, returns 0 if no, 1 if yes 
        function measure_charge_status()
            return bat_charge_status
        end
    
        substate=0 -- sub state for state machine of message
    
        -- battery management
        function update_battery()
            dt=sim.getSimulationTimeStep()
            battery=battery-factLighting*lighting-factMoving*moving-factCPU*cpu
            --sim.setGraphUserData(BatGraphHandle,"Battery",battery)  -- should be uncommented only for one robot
        end
    
        delay_start=sim.getSimulationTime()
    
            -- wait for x milliseconds  global variable delay_start should be initialised with:  delay_start=sim.getSimulationTime()
        function _delay_ms(x)
            --sim.wait(x/1000.0,true)
                if ((sim.getSimulationTime()-delay_start)>=(x/1000.0)) then
                    return 1
                end
            return 0
        end
    
    
    
        -------------------------------------------------
        -- not implemented functions:
    
        -- Returns the value of ambient light
        --  note: will return -1 if there is an incoming message (which also uses a/d)
        --  note: turns off interrupts for a short while to sample a/d 
        function get_ambient_light()
            return -1
        end
    
        -------------------------------------------------
        -- other initialisations
    
        robot_id = math.random(0, 255) -- set robot id
    
        -- get number of other robots
    
        NUMBER_OTHER_ROBOTS=0
        objIndex=0
        while (true) do
            h=sim.getObjects(objIndex,sim.object_shape_type)
            if (h<0) then
                break
            end
            objIndex=objIndex+1
            --sim.addLog(sim.verbosity_msgs,"objIndex: "..objIndex)
            if ((sim.readCustomDataBlock(h,"kilobot")=="kilobot") and (KilobotHandle ~= h))then
                NUMBER_OTHER_ROBOTS=NUMBER_OTHER_ROBOTS+1
                --sim.addLog(sim.verbosity_msgs,"NUMBER_OTHER_ROBOTS: "..NUMBER_OTHER_ROBOTS)
            end
        end    
    
        --sim.addLog(sim.verbosity_msgs,"number of robots found: "..robotnb)
end
------------------------------------------------------------------------------ 
-- Following few lines automatically added by CoppeliaSim to guarantee compatibility 
-- with CoppeliaSim 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
  local version=sim.getInt32Param(sim.intparam_program_version) 
  local revision=sim.getInt32Param(sim.intparam_program_revision) 
  if (version<30104)and(revision<3) then 
      return _aShapeHandle_ 
  end 
  return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 

function sysCall_cleanup() 
    set_color(0,0,0)
end 

function sysCall_actuation() 

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- main script loop
    
    update_battery() -- update battery value
    
    receive_data() -- received data by ir
    
    send_data() -- send data by ir
    
    --special message controller, handles controll messages like sleep and resume program
    if(special_mode==1) then
    
        run_program=0
    
        special_mode=0
        set_motor(0,0)
    
        -- modes for different values of special_mode_message     
        --0x01 bootloader (not implemented)
        --0x02 sleep (not implemented)
        --0x03 wakeup, go to mode 0x04
        --0x04 Robot on, but does nothing active
        --0x05 display battery voltage
        --0x06 execute program code
        --0x07 battery charge
        --0x08 reset program
    
    
        if(special_mode_message==0x02) then
          -- sleep    
            wakeup=0
            --enter_sleep();//will not return from enter_sleep() untill a special mode message 0x03 is received    
        elseif((special_mode_message==0x03)or(special_mode_message==0x04)) then
          --wakeup / Robot on, but does nothing active
            enable_tx=0
            
            -- make the led blink
            if (substate==0) then    
                set_color(3,3,0)
                substate=substate+1
                delay_start=sim.getSimulationTime()
            elseif (substate==1) then
                if (_delay_ms(50)==1) then 
                    substate=substate+1
                end
            elseif (substate==2) then
                set_color(0,0,0)
                substate=substate+1
                delay_start=sim.getSimulationTime()
            elseif (substate==3) then
                if (_delay_ms(1300)==1) then
                    substate=0
                end
            end
    
            enable_tx=1
            special_mode=1
        
        elseif(special_mode_message==0x05) then
         -- display battery voltage
            enable_tx=0
    
            if(measure_voltage()>400) then
                set_color(0,3,0)
            elseif(measure_voltage()>390) then
                set_color(0,0,3)
            elseif(measure_voltage()>350) then
                set_color(3,3,0)
            else
                set_color(3,0,0)
            end
    
            enable_tx=1
        elseif (special_mode_message==0x06) then
            --execute program code
            enable_tx=1
            run_program=1
            substate = 0
            --no code here, just allows special_mode to end 
    
        elseif (special_mode_message==0x07) then
         --battery charge
            enable_tx=0
            --if(measure_charge_status()==1) then
            
            if (battery<charge_max) then
                if (substate==0) then    
                    set_color(1,0,0)
                    substate=substate+1
                    delay_start=sim.getSimulationTime()
                elseif (substate==1) then
                    if (_delay_ms(50)==1) then 
                        substate=substate+1
                    end
                elseif (substate==2) then
                    set_color(0,0,0)
                    substate=substate+1
                    delay_start=sim.getSimulationTime()
                elseif (substate==3) then
                    if (_delay_ms(300)==1) then
                        substate=0
                    end
                end
            
                battery=battery+charge_rate
            
                if (battery>battery_init) then
                    battery=battery_init
                end
            end
            special_mode=1
    
    
    
            enable_tx=1
        elseif (special_mode_message==0x08) then
            
            if (reset_substate==0)    then
            --reset
            enable_tx=0
            run_reset()
            run_program = 0
            special_mode_message = 0x08
            reset_substate = reset_substate + 1
            -- wait some time for stopping messages
            delay_reset=sim.getSimulationTime()
            elseif (sim.getSimulationTime()-delay_reset>=1.5) then      
                special_mode_message = 3
                reset_substate    = 0
            else
                while (sim.receiveData(0,"Message",MsgSensorsHandle)) do 
                end
            end
            
            special_mode = 1
    
        end
    
    end
    
    if(run_program== 1) then
        
        user_prgm()
    
    end
    

end 
 