local historyIncreasing = {1,1,1,1,1,1,5}
local historyDecreasing = {2,2,2,2,2,2,2,1}
local derivitveHistory = {}

-- numerical derivitive of array
local function getDerivitiveOfArray(array)
    local derivitive ={}
    for i = 1,#array-1 do 
        derivitive[i] = array[i+1] - array[i]
    end
    return derivitive
end

--updates history with distance value, and remove oldest value
function updateDistanceHistory(distanceHistory, newValue)
    local length = #distanceHistory
    table.remove(distanceHistory,length)
    table.insert(distanceHistory,newValue)
    -- table.insert(historicalSum,sumArray(history))
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

function hotterColder (array)
    derivitive = getDerivitiveOfArray(array)
    if(sumArray(derivitive) < 0) then 
        print('increaseing')
        -- set choice based on current choice if no choice set make random choice from which make choice based on history 
        
        -- if there is a distance increase history, then calculate the rate of distance increasing based on sample history, determine direction of travel based on rate of increase. 
    else
        print('decreasing')
        
        -- we are on the right path, now monitor history of rate of distance decrease. optimise motor speed to maximized rate of distance decrease. 
        
        -- i need an equation that can be optmized. 
        
        -- rate of decrease is a function of the ratio of both motors set at max speed 
        
    
    end
    
end

-- update derivitive history, if empty then increase to desired length else remove oldest entry and add new
local function updateDerivitiveHistory(derHistory,newHistoryArray)
    if(#derHistory<6) then
        table.insert(derHistory,#derHistory+1, newHistoryArray)
    else
        table.remove(derHistory,1);
        table.insert(derHistory,#derHistory,newHistoryArray)
    end
end

-- show if derivitive is changin if distance is increaseing 
for i=0,10 do
    updateDistanceHistory(historyDecreasing,i)
    updateDistanceHistory(historyIncreasing,1)
    print('--------------------')
    print('decreasing is now:')
    hotterColder(historyDecreasing)
    print('increasing is now:')
    hotterColder(historyIncreasing)
    print('--------------------')
end







