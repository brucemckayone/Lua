local history = {1,2,3,4,5,6,7,8}
local leftMotorIntensity = 200;
local rightMotorIntensity = 200;
local motorDirectionRatio = 0
local currentTravelingDirection = false; --left is false right is true
local currentDerivitive = {}
local derivitveHistory = {}


local function setDirection(direction, intensity)
    intensity = math.abs(intensity)
    if(not direction) then
        leftMotorIntensity = leftMotorIntensity*intensity
    elseif (direction) then
        rightMotorIntensity= intensity*rightMotorIntensity
    end
    print(direction)
    print(intensity);
end

-- numerical derivitive of array
local function getDerivitiveOfArray(array)
    local derivitive ={}
    for i = 1,#array-1 do 
        derivitive[i] = array[i+1] - array[i]
    end
    return derivitive
end

--updates history with distance value, and remove oldest value
local function rollArray(distanceHistory, newValue)
    local length = #distanceHistory
    table.remove(distanceHistory,length)
    table.insert(distanceHistory,1,newValue)
    return distanceHistory
end

--get summation of array
local function sumArray(array)
    local s = 0
    for i=1,#array do
        s=s+array[i]
    end
    return s
end

-- implementatino of print_r form PHP, using this to get nicer values
local function print_r(arr, indentLevel)
    local str = ""
    local indentStr = "#"

    if(indentLevel == nil) then
        print(print_r(arr, 0))
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

local function linSpace(length)
    local r = {}
    for i = 1,length do
        table.insert(r,i);
    end
    return r;
end


-- update derivitive history, if empty then increase to desired length else remove oldest entry and add new
local function updateDerivitiveHistory(derHistory,newHistoryArray)
    if(#derHistory<30) then
        table.insert(derHistory,#derHistory+1, newHistoryArray)
    else
        table.remove(derHistory,1);
        table.insert(derHistory,#derHistory,newHistoryArray)
    end
    return derHistory
end

local function chooseDirection(derHistory)
    local length = #derHistory
    local x = {}
    local y = linSpace(length);

    for i = 1,length do
        local xi = sumArray(derHistory[i])/length
        table.insert(x,xi)
    end
    X = sumArray(x)/#x
    Y = sumArray(y)/#y

    local top = {}
    local bottom = {}
    for i =1,length do
       table.insert( top, (x[i]-X) * (y[i]-Y ) )
       table.insert( bottom, ((x[i]-X))^2)
    end
   return sumArray(top)/sumArray(bottom) --slope
end

local function determinePath(derHistory,count)
    if(count % 30 == 0) then
        local rateOfRelativePostionChange = chooseDirection(derivitveHistory)
        print(rateOfRelativePostionChange)
    
        if(rateOfRelativePostionChange<0) then
            --your moving away from neighbour change direction with intensity calculated
            setDirection(not currentTravelingDirection,rateOfRelativePostionChange)
        else
            --you are moving towards neighbour adjust intensity of turn
            setDirection(currentTravelingDirection,rateOfRelativePostionChange)
        end
    end
end

local count = 0
for i=1,200 do
    count=count+1;
    --simulate noisy periodical changes in distance of parent to child
    local change = math.floor(i % 4 * ( i % 2*i ) / 8)
    history = rollArray(history,change)
    currentDerivitive = getDerivitiveOfArray(history)
    derivitveHistory = updateDerivitiveHistory(derivitveHistory,currentDerivitive);
    determinePath(derivitveHistory,count)
end






