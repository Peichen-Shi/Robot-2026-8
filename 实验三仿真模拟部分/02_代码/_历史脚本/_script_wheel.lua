sim = require('sim')

function sysCall_init() 
    rolling = sim.getObject("..")
    slipping=sim.getObjectChild(sim.getObjectChild(rolling, 0), 0)
    wheel=sim.getObjectChild(slipping, 0)
    --print(sim.getObjectName(rolling), sim.getObjectName(slipping), sim.getObjectName(wheel))
end
-- Following script resets the second joint on the omni-wheel (important to achieve the desired omni-wheel effect)


function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    sim.resetDynamicObject(wheel)
    sim.setObjectPosition(slipping+sim.handleflag_reljointbaseframe,rolling,{0,0,0})
    sim.setObjectOrientation(slipping+sim.handleflag_reljointbaseframe,rolling,{0,-math.pi/4,0})
    sim.setObjectPosition(wheel+sim.handleflag_reljointbaseframe,rolling,{0,0,0})
    sim.setObjectOrientation(wheel+sim.handleflag_reljointbaseframe,rolling,{0,0,0})
end 
