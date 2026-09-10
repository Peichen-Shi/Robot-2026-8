sim = require('sim')
simIK = require('simIK')

State = {
   open = 1,
   close = 2,
   pause = 0
}

function sysCall_init()
    -- do some initialization here

    h = sim.getObject('../Prismatic_joint')
    handle = sim.getObject('..')
    connector=sim.getObject('../attachPoint')
    objectSensor=sim.getObject('../attachProxSensor')
    left=sim.getObject('../left_gripper_5_respondable')
    right=sim.getObject('../right_gripper_5_respondable')
    base_1=sim.getObject('../Cuboid')
    base_2=sim.getObject('../Cuboid0')

    current_state_signal = 'signal.state'
    target_state_signal = 'signal.target'
    
    attachedShape = nil
    sim.setIntProperty(handle, current_state_signal, State.pause)
    sim.setIntProperty(handle, target_state_signal, State.pause)
    speed = 0.01
    
    ikEnv=simIK.createEnvironment() 

    ikGroup_damped=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,0.01,5)
    
    ik_base_1 = simIK.createDummy(ikEnv)
    ik_base_2 = simIK.createDummy(ikEnv)
    simIK.setObjectMatrix(ikEnv,ik_base_1,-1,sim.getObjectMatrix(base_1,-1)) 
    simIK.setObjectMatrix(ikEnv,ik_base_2,-1,sim.getObjectMatrix(base_2,-1)) 
    
    js = {1, 2, 4, 5, 6, 7} 
    local ds = {3, 6, 8} 
    sim_joints = {}
    ik_joints = {}
    for _, side in ipairs({'left', 'right'}) do
        --print(side)
        local ikJoints = {}
        local parent = ik_base_2
        --print('Create IK joints')
        for i, j in ipairs(js) do
            local joint = sim.getObject('../'..side..'_gripper_'..j)
            --print(j, joint)
            -- create a joint in the IK environment: 
            ikJoints[j]=simIK.createJoint(ikEnv,simIK.jointtype_revolute)
            -- set it into IK mode: 
            simIK.setJointMode(ikEnv,ikJoints[j],simIK.jointmode_ik) 
            -- set the same joint limits as its CoppeliaSim counterpart joint:
            local cyclic,interv=sim.getJointInterval(joint)
            simIK.setJointInterval(ikEnv,ikJoints[j],cyclic,interv)
            -- set the same joint position as its CoppeliaSim counterpart joint: 
            simIK.setJointPosition(ikEnv,ikJoints[j],sim.getJointPosition(joint))
            -- set the same object pose as its CoppeliaSim counterpart joint: 
            simIK.setObjectMatrix(ikEnv,ikJoints[j],-1,sim.getObjectMatrix(joint,-1))
            -- set its corresponding parent: 
            simIK.setObjectParent(ikEnv,ikJoints[j],parent,true) 
            parent=ikJoints[j]
            table.insert(sim_joints,joint)
            table.insert(ik_joints,ikJoints[j])
        end
        
        --print('Create IK links')
        for i, d in ipairs(ds) do
            
            
            local dum=sim.getObjectHandle('../dummy_'..side..'_gripper_'..d)
            local dumA=sim.getObjectHandle('../dummy_'..side..'_gripper_'..d..'a')
            local ik_dum = simIK.createDummy(ikEnv)
            local ik_dumA = simIK.createDummy(ikEnv)
            simIK.setObjectMatrix(ikEnv,ik_dum,-1,sim.getObjectMatrix(dum,-1))
            simIK.setObjectParent(ikEnv,ik_dum,ikJoints[d-1], true)
            simIK.setObjectMatrix(ikEnv,ik_dumA,-1,sim.getObjectMatrix(dumA,-1))
            simIK.setObjectParent(ikEnv,ik_dumA,ik_base_1, true)
            simIK.setLinkedDummy(ikEnv,ik_dumA,ik_dum)
            local c
            if d ~=6 then
                c = simIK.constraint_x + simIK.constraint_y
            else
                c = simIK.constraint_gamma
            end
            local ikElementHandle=simIK.addElement(ikEnv,ikGroup_damped,ik_dum)
            simIK.setElementBase(ikEnv,ikGroup_damped,ikElementHandle,ik_base_2)
            simIK.setElementConstraints(ikEnv,ikGroup_damped,ikElementHandle,c)
            ikElementHandle=simIK.addElement(ikEnv,ikGroup_damped,ik_dumA)
            simIK.setElementBase(ikEnv,ikGroup_damped,ikElementHandle,ik_base_1)
            simIK.setElementConstraints(ikEnv,ikGroup_damped,ikElementHandle,c)
        end
    end
    
    min_joint_position = -0.0229
    max_joint_position = 0.00098

    --print('Done')
end

function sysCall_actuation()
    -- put your actuation code here
    
    local state = sim.getIntProperty(handle, current_state_signal)
    local target = sim.getIntProperty(handle, target_state_signal)
    
    if state == target then
        sim.setJointTargetVelocity(h, 0)
        return
    end
    if target == State.pause then
        sim.setJointTargetVelocity(h, 0)
        sim.setIntProperty(handle, current_state_signal, State.pause)
        return
    end
    
    simIK.setObjectMatrix(ikEnv,ik_base_1,-1,sim.getObjectMatrix(base_1,-1)) 
    simIK.setObjectMatrix(ikEnv,ik_base_2,-1,sim.getObjectMatrix(base_2,-1)) 
    simIK.handleGroup(ikEnv,ikGroup_damped)
    --print('handleIkGroup', simIK.handleGroup(ikEnv,ikGroup_damped))
    for i = 1, #sim_joints do
        sim.setJointPosition(sim_joints[i],simIK.getJointPosition(ikEnv,ik_joints[i]))
    end
    
    if target == State.open then
        sim.setJointTargetVelocity(h, speed)
        if(attachedShape) then
            sim.setObjectParent(attachedShape,-1,true)
            sim.resetDynamicObject(attachedShape)
            attachedShape = nil
            --print('Detached shape')
        end
        local ext = sim.getJointPosition(h)
        --print('opening', ext)
        if(ext > max_joint_position) then
            sim.setIntProperty(handle, current_state_signal, State.open)
        end
        return
    end
    
    sim.setJointTargetVelocity(h, -speed)  
    local ext = sim.getJointPosition(h)
    --print('closing', ext)
    if(ext < min_joint_position) then
        sim.setIntProperty(handle, current_state_signal, State.close)
        return
    end
    
    index=0
    while true do
        shape=sim.getObjects(index,sim.object_shape_type)
        if (shape==-1) then
            break
        end
        local r, v = sim.getObjectInt32Parameter(shape,sim.shapeintparam_static)
        if(v) then
            r, v = sim.getObjectInt32Parameter(shape,sim.shapeintparam_respondable)
            if (v~=0 and sim.checkProximitySensor(objectSensor,shape)==1) then
                if(sim.checkCollision(left, shape) ~= 0 and  sim.checkCollision(right, shape) ~= 0) then
                    --print("Ok, we found a non-static respondable shape that was detected")
                    attachedShape=shape
                    -- Do the connection:
                    sim.setObjectParent(attachedShape,connector,true)
                    sim.setIntProperty(handle, current_state_signal, State.close)
                    sim.setJointTargetVelocity(h, 2 * speed)
                    sim.resetDynamicObject(shape)
                    print('Attached shape')
                    break
                end
            end
        end
        index=index+1
    end
    
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    --simIK.eraseEnvironment(ikEnv) 
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
