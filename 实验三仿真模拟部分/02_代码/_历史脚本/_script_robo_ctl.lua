simRobomaster = require('simRobomaster')
sim = require('sim')

function sysCall_init()
    local handle = sim.getObject('..')
    rm_handle = simRobomaster.create_ep(handle, "/0", "RM0001")  
end