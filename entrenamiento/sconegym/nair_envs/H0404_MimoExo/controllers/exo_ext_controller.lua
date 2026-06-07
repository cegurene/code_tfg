-- Lue controller for spasticity conditions
-- Designed by Andres Chavarrias, NAIR Group
-- V0 17/03/2025

function init(model, par)
	-- Find join DoF 
	knee = model:find_dof("knee_angle_r")
	knee_angle = math.abs(knee:position()) -- position
	knee_vel = math.abs(knee:velocity())
	-- Find exo actuator
	exo_motor_r	= model:find_actuator("exo_rotation")
	scone.debug("exo_motor_r: "..exo_motor_r:name())
	
	uo	= 0
	
	

	-- ****************************** David **********************************************
	
end


function update(model)

	-- getting current simulation time
	local t = model:time()
	-- Spasticity parameters
	local uo = 1
	if t >= 0 then
		uo = 50.0
	end
	
    exo_motor_r:add_input(uo)

	-- scone.debug("musbfsh_length: "..musbfsh_length)
	-- scone.debug("delbfsh_length: "..delbfsh_length)
	


	
	
end