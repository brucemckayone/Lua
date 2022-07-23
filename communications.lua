


local sim ={}


--creates an object with Kilobot properties. 
-- local Kilobot={
--     half_diam=0.0165,
--     RATIO_MOTOR = 10/255, -- ratio for getting 100% = 1cm/s or 45deg/s

--     -- 4 constants that are calibration values used with the motors
--     cw_in_place     = 200, -- value for cw motor to turn the robot cw in place (note: ccw motor should be off)
--     ccw_in_place    = 200, -- value for ccw motor to turn the robot ccw in place (note: cw motor should be off) 
--     cw_in_straight  = 200, -- value for the cw motor to move the robot in the forward direction
--     ccw_in_straight = 200, -- value for the ccw motor to move the robot in the forward direction 
    
    
--     -- for battery management
--     battery_init = 8000000,  -- battery initial charged value
--     battery =  8000000,    -- battery simulator (~5h with 2 motors and 3 colors at 1 level)
--     factMoving = 100,        -- factor for discharging during moving
--     moving = 0,                -- for battery discharging    1: one motor, 2: 2 motors, 0: not moving
--     factCPU = 10,            -- factor for discharging during cpu
--     cpu=1,                    -- cpu state: sleep = 0, active = 1
--     factLighting = 40,     -- factor for discharging during lighting
--     lighting=0,     -- for battery managing
--     bat_charge_status=0  ,     -- battery charge status 0= no, 1, yes    
--     charge_rate=400,       -- charge rate
--     charge_max = 8000000, -- battery_init*99.9/100 -- end of charge detection

--     init  = function()
--         local i=0
--         local foundCtrller = false
--         -- is kilobot in range of controller
--         while true do
--             local h = sim.getObjects(i,sim.handle_all)
--             if h==-1 then break end
--             if sim.readCustomDataBlock(h,"kilobot")=='kilobotcontroller' then
--                 foundCtrller=true
--                 break
--             end
--             i=i+1
--         end
--         if not foundCtrller then
--             sim.addLog(sim.verbosity_scripterrors,"The KiloBot could not find a controller. Make sure to have exactly one 'Kilobot_Controller' model in the scene.") 
--         end
--         -- add handles
--         KilobotHandle=sim.getObject('.') 
    
--         LeftMotorHandle=sim.getObject('./revolute_jointLeftLeg')
--         RightMotorHandle=sim.getObject('./revolute_jointRightLeg')
    
--         MsgSensorsHandle=sim.getObject('./msgSensor')
    
--         sensorHandle=sim.getObject("./proximity_sensor")
--         BaseHandle=sim.getObject("./BatHold")

--         obstacles=sim.createCollection(0)
--         sim.addItemToCollection(obstacles,sim.handle_all,-1,0)
--         sim.addItemToCollection(obstacles,sim.handle_tree,KilobotHandle,1)
--     end
    
-- }
-- Kilobot.init()

local Communication = {
    -- received message
    message_rx = {0,0,0,0,0,0},
    enable_tx = 0,-- to turn on/off the transmitter
    senderID = nil,
    data = nil,
    special_mode = 1,
    run_program = 0,
    special_mode_message = 3,
    dataName = nil,
    dataHeader = nil,

    -- receive latest message and process it
    receiveData = function(self)
        self.data, self.senderID, self.dataHeader,self.dataName= sim.receiveData(0, "Message" , MsgSensorsHandle)
        if (self.data ~= nil) then
            local senderHandle = sim.getObjectAssociatedWithScript(self.senderID)
            local udata = sim.unpackInt32Table(self.data)
            self.message_rx[1]=udata[1]
            self.message_rx[2]=udata[2]
            self.message_rx[3]=udata[3]

            -- special message
            if (self.message_rx[3] == 0x1) then
                self.special_mode_message=self.message_rx[2]
                self.special_mode = 1

            end
        end
    end,

    --sending data 
    irstart = sim.getSimulationTime(),

    
    sendData = function (self) -- send data from ir every 0.2s, at a max distance of 7cm
        local newir=sim.getSimulationTime()
        if ((self.enable_tx==1) and (newir-self.irstart>0.2)) then
            sim.sendData(sim.handle_all,0,"Message",sim.packInt32Table({self.tx0,self.tx1,self.tx2}),MsgSensorsHandle,0.07,3.1415,3.1415*2,0.8)
            self.irstart=newir
        end
    end,


    -- get message
    getMessage = function (self)
            if (self.data ~= nil) then
            local udata=sim.unpackInt32Table(self.data)
            self.message_rx[1]=udata[1]
            self.message_rx[2]=udata[2]
            self.message_rx[3]=udata[3]
            
            local result,distance,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.checkProximitySensor(sensorHandle,senderHandle)
            if (result == 1) then
                self.message_rx[4]=(distance+Kilobot.half_diam)*1000  -- distance in mm + 1/2diameter of robo
                self.message_rx[6]=1    -- message receveid
            else
                self.message_rx[6]=0 -- no message receveid
            end            
        else
            self.message_rx[6]=0 -- no message receveid
        end
    end,
    
    -- message sending
    tx0=0,
    tx1=0,
    tx2=0,
    messageOut = function (self,_tx0,_tx1,_tx2)
        self.tx0=_tx0
        self.tx1=_tx1
        self.tx2=_tx2    
    end

}