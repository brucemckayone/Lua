local Locomotion = {
    motorMaxIntensity =255,
    motorIntensity = 150,
    motorRatio =0.5,

    -- set motor ratio used for turning--0.5 is straight lower is left higher is right
    setMotorRatio = function(self,ratio)
        self.motorRatio = ratio
        local leftMotorRatio = 1-self.motorRatio;
        set_motor(150*leftMotorRatio,150*self.motorRatio)
    end,
    -- sets intensity of motor
    setMotorIntensity =  function(self,intensity)
        if(intensity> self.motorMaxIntensity) then intensity= self.motorMaxIntensity end
        self.motorIntensity = intensty

    end,
    
    stopMotor = function(self)set_motor(0,0)end,

    zigZagMotorSubstrate = 0,
    doNoisyZigZagWalk = function(self)
        if(self.zigZagMotorSubstrate == 0 )then
            self.zigZagMotorSubstrate=1,
            setDelayStartAndDoFunction(function()self:setMotorRatio(0.5)end) --go forward reset clock
        elseif (self.zigZagMotorSubstrate== 1) then
            randomDelay =math.random(1000,10000)
            self.zigZagMotorSubstrate=asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,2)
        elseif (self.zigZagMotorSubstrate== 2) then
            self.zigZagMotorSubstrate = 3;
            setDelayStartAndDoFunction(function()self:setMotorRatio(0.9)end) -- go hard right restart clock
        elseif (self.zigZagMotorSubstrate==3) then
        randomDelay =math.random(1000,10000)
            self.zigZagMotorSubstrate=    asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,4)
        elseif (self.zigZagMotorSubstrate==4) then
            self.zigZagMotorSubstrate= 5;
            setDelayStartAndDoFunction(function()self:setMotorRatio(0.1)end) --go left restart clock
        elseif(self.zigZagMotorSubstrate== 5)then
        randomDelay =math.random(1000,10000)
           self.zigZagMotorSubstrate= asyncAwaitReturnSubstrate(self.zigZagMotorSubstrate,randomDelay,0)
        end
    end,
    
    
    --spriral search
    spiralSubstrate = 0,
    doSpiralSearch = function(self)
        self.spiralSubstrate = self.spiralSubstrate + 1
        local spiralExpansionCount = 7000
        self:setMotorRatio(self.spiralSubstrate/spiralExpansionCount)
        if(self.spiralSubstrate == spiralExpansionCount)then self.spiralSubstrate = 0 end
    end,
    
    
    ----------------
    leftMotorIntensity = 60,
    rightMotorIntensity = 60,
    setDirection = function(self,direction, intensity)
        local intensity = math.abs(intensity)
        print(intensity)
        if( direction) then
            self.leftMotorIntensity = utils:restrictNumberLimits( intensity, 255)
            self.rightMotorIntensity= utils:restrictNumberLimits( intensity/2, 255)
        elseif (not direction) then
            self.rightMotorIntensity= utils:restrictNumberLimits(  intensity* self.rightMotorIntensity,255)
            self.leftMotorIntensity = utils:restrictNumberLimits( intensity/2,255)
        end
        set_motor(self.leftMotorIntensity,self.rightMotorIntensity)
    end,
    -------------------------------------------------
    sampleLength = 200,
    derHistory= {0,0,0,0},
    distanceHistory={0,0,0,0},
    sampleCount = 0,
    currentTurningDirection = false,
    
    --------------------REDUCE DISTANCE BETWEEN SELF AND TARGET -------------------------
    minDistance  = function (self,distance)
            self.sampleCount = self.sampleCount +1
            self.distanceHistory = arrayUtils.roll(self.distanceHistory,distance, self.sampleLength)
            
            --dbug:print_r(self.distanceHistory)
            local derHistorySample = arrayUtils.diff(self.distanceHistory)
        
            self.derHistory = arrayUtils.roll(self.derHistory, derHistorySample, self.sampleLength)
            
            -- when a full sample is ganthered

            if(self.sampleCount==self.sampleLength)then 
                self.sampleCount= 0
                
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
                local topSum = arrayUtils.sum(top ) 
                local bottomSum=arrayUtils.sum(bottom)
                self.rateOfRelativePostionChange = topSum /(bottomSum*10000) --slope
                --print(self.rateOfRelativePostionChange)
                --print(self.rateOfRelativePositionChange)
                self.rateOfRelativePostionChange = utils:restrictNumberLimits(self.rateOfRelativePostionChange,500)
                
                if( self.rateOfRelativePostionChange < 0 ) then
                    self:setDirection(  not self.currentTurningDirection,self.rateOfRelativePostionChange)
                else
                    self:setDirection( self.currentTurningDirection,self.rateOfRelativePostionChange)
                end
            end
    end,
    
    --------------------ACT AS FOLLOWER -------------------------
    actAsFollower = function(self,distance)
        -- if distance is reveived
        if (distance ~= nil)  then
            local isSearching = false
            distance= distance + half_diam
                if sim.readCustomDataBlock(detectedObjectHandle,"kilobot")=="detectable" then
                    local initSubstrate =0
                    if(initSubstrate == 0 )then
                        initSubstrate = 1
                        self:minDistance(distance)
                    end
                    if (distance < 0.035) then isSearching= false; self:stopMotor(); print('is stopping') end
                    if (distance > 0.08) then isSearching= true end
                    
                    if(isSearching)then 
                        self:minDistance(distance)
                        set_color(0,5,0)
                    else
                        self:stopMotor()
                        set_color(0,0,5)
                    end
                end
            else
                self.spiralSubstrate=0
                self:stopMotor()
                set_color(5,0,0)
                self:doSpiralSearch()
            end
    end,

    actAsLeader= function(self,distance)
        local isSearching = 0
        if (distance ~= nil)  then 
            distance= distance + half_diam
            --print(distance)
            if sim.readCustomDataBlock(detectedObjectHandle,"kilobot")=="detectable" then
                isSearching =true
                if (distance> 0.08 ) then isSearching= false end
                if (distance< 0.01) then isSearching= true end
                if(isSearching)then
                    self:doNoisyZigZagWalk() 
                else
                    self:stopMotor()
                    set_color(0,0,5)
                end
                --if(distance > 0.09)then set_motor(0,0) end
                --if(distance< 0.025) then  end
            end
        end
    end
}