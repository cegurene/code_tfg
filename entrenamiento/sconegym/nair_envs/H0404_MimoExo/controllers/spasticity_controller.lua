-- Lua controller for spasticity conditions
-- Designed by Andres Chavarrias, NAIR Group
-- V0 17/03/2025 

function init(model, par)
	-- Find join DoF 
	knee = model:find_dof("knee_angle_r")
	knee_angle = math.abs(knee:position()) -- position
	knee_vel = math.abs(knee:velocity())
	-- Find muscles activation
	-- Muscles actuators for rigth leg
	hm_r	= model:find_actuator("hamstrings_r")
	scone.debug("hm_r: "..hm_r:name())
	bfsh_r	= model:find_actuator("bifemsh_r")
	scone.debug("bfsh_r: "..bfsh_r:name())
	gm_r	= model:find_actuator("glut_max_r")
	scone.debug("gm_r: "..gm_r:name())
	ip_r	= model:find_actuator("iliopsoas_r")
	scone.debug("ip_r: "..ip_r:name())
	rf_r	= model:find_actuator("rect_fem_r")
	scone.debug("rf_r: "..rf_r:name())
	vi_r 	= model:find_actuator("vas_int_r")
	scone.debug("vi_r: "..vi_r:name())
	gs_r	= model:find_actuator("gastroc_r")
	scone.debug("gs_r: "..gs_r:name())
	sl_r 	= model:find_actuator("soleus_r")
	scone.debug("sl_r: "..sl_r:name())
	ta_r	= model:find_actuator("tib_ant_r")
	scone.debug("ta_r: "..ta_r:name())
	-- Muscles actuators for left leg
	hm_l	= model:find_actuator("hamstrings_l")
	scone.debug("hm_l: "..hm_l:name())
	bfsh_l	= model:find_actuator("bifemsh_l")
	scone.debug("bfsh_l: "..bfsh_l:name())
	gm_l	= model:find_actuator("glut_max_l")
	scone.debug("gm_l: "..gm_l:name())
	ip_l	= model:find_actuator("iliopsoas_l")
	scone.debug("ip_l: "..ip_l:name())
	rf_l	= model:find_actuator("rect_fem_l")
	scone.debug("rf_l: "..rf_l:name())
	vi_l 	= model:find_actuator("vas_int_l")
	scone.debug("vi_l: "..vi_l:name())
	gs_l	= model:find_actuator("gastroc_l")
	scone.debug("gs_l: "..gs_l:name())
	sl_l 	= model:find_actuator("soleus_l")
	scone.debug("sl_l: "..sl_l:name())
	ta_l	= model:find_actuator("tib_ant_l")
	scone.debug("ta_l: "..ta_l:name())
	
	-- find muscles information
	-- Rigth leg
	mushm_r	= model:find_muscle("hamstrings_r")
	act_hm_r = mushm_r:activation()
	scone.debug("mushm_r: "..mushm_r:name())

	musbfsh_r	= model:find_muscle("bifemsh_r")
	act_bfsh_r = musbfsh_r:activation()
	scone.debug("musbfsh_r: "..musbfsh_r:name())

	musgm_r	= model:find_muscle("glut_max_r")
	act_gm_r = musgm_r:activation()
	scone.debug("musgm_r: "..musgm_r:name())

	musip_r	= model:find_muscle("iliopsoas_r")
	act_ip_r = musip_r:activation()
	scone.debug("musip_r: "..musip_r:name())

	musrf_r	= model:find_muscle("rect_fem_r")
	act_rf_r = musrf_r:activation()
	scone.debug("musrf_r: "..musrf_r:name())

	musvi_r	= model:find_muscle("vas_int_r")
	act_vi_r = musvi_r:activation()
	scone.debug("musvi_r: "..musvi_r:name())

	musgs_r	= model:find_muscle("gastroc_r")
	act_gs_r = musgs_r:activation()
	scone.debug("musgs_r: "..musgs_r:name())

	mussl_r	= model:find_muscle("soleus_r")
	act_sl_r = mussl_r:activation()
	scone.debug("mussl_r: "..mussl_r:name())

	musta_r	= model:find_muscle("tib_ant_r")
	act_ta_r = musta_r:activation()
	scone.debug("musta_r: "..musta_r:name())

	-- Left leg
	mushm_l	= model:find_muscle("hamstrings_l")
	act_hm_l = mushm_l:activation()
	scone.debug("mushm_l: "..mushm_l:name())

	musbfsh_l	= model:find_muscle("bifemsh_l")
	act_bfsh_l = musbfsh_l:activation()
	scone.debug("musbfsh_l: "..musbfsh_l:name())

	musgm_l	= model:find_muscle("glut_max_l")
	act_gm_l = musgm_l:activation()
	scone.debug("musgm_l: "..musgm_l:name())

	musip_l	= model:find_muscle("iliopsoas_l")
	act_ip_l = musip_l:activation()
	scone.debug("musip_l: "..musip_l:name())

	musrf_l	= model:find_muscle("rect_fem_l")
	act_rf_l = musrf_l:activation()
	scone.debug("musrf_l: "..musrf_l:name())

	musvi_l = model:find_muscle("vas_int_l")
	act_vi_l = musvi_l:activation()
	scone.debug("musvi_l: "..musvi_l:name())

	musgs_l	= model:find_muscle("gastroc_l")
	act_gs_l = musgs_l:activation()
	scone.debug("musgs_l: "..musgs_l:name())

	mussl_l	= model:find_muscle("soleus_l")
	act_sl_l = mussl_l:activation()
	scone.debug("mussl_l: "..mussl_l:name())

	musta_l	= model:find_muscle("tib_ant_l")
	act_ta_l = musta_l:activation()
	scone.debug("musta_l: "..musta_l:name())
	
	-- init muscles activation with STD
	-- Rigth leg
	O_hm_r		= par:create_from_mean_std(hm_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_bfsh_r	= par:create_from_mean_std(bfsh_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_gm_r		= par:create_from_mean_std(gm_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_ip_r		= par:create_from_mean_std(ip_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_rf_r		= par:create_from_mean_std(rf_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_vi_r		= par:create_from_mean_std(vi_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_gs_r		= par:create_from_mean_std(gs_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_sl_r		= par:create_from_mean_std(sl_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_ta_r		= par:create_from_mean_std(ta_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	
	-- Left leg
	O_hm_l		= par:create_from_mean_std(hm_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_bfsh_l	= par:create_from_mean_std(bfsh_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_gm_l		= par:create_from_mean_std(gm_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_ip_l		= par:create_from_mean_std(ip_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_rf_l		= par:create_from_mean_std(rf_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_vi_l		= par:create_from_mean_std(vi_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_gs_l		= par:create_from_mean_std(gs_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_sl_l		= par:create_from_mean_std(sl_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_ta_l		= par:create_from_mean_std(ta_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	
	-- ****************************** David **********************************************
	Lo_bfsh_r 	= par:create_from_mean_std(bfsh_r:name() .. "-length_offset", 0.0, 0.1, 0.0, 0.7)
	gl1 		= par:create_from_mean_std("gl1", 0.15, 0.1, 0.0, 1.0)
	gl2 		= par:create_from_mean_std("gl2", 0.15, 0.1, 0.0, 1.0)
	act1 		= par:create_from_mean_std("act1", 0.4, 0.1, 0.0, 1.0)
	act2 		= par:create_from_mean_std("act2", 0.4, 0.1, 0.0, 1.0)
	uo	= 0
	
	-- buffers
	bfsh_h = {}
	time_h = {}
	
	dt = 0.001
	delay = 0.01
	n = 10
	
	for i = 1, n do
		
		bfsh_h[i] = 0
		time_h[i] = 0
		
	end
	-- ****************************** David **********************************************
	
end

function sigmoid(x)
    return 1 / (1 + math.exp(-x))
end

function clip(x, min_val, max_val)
    if x < min_val then
        return min_val
    elseif x > max_val then
        return max_val
    else
        return x
    end
end

function add_spas_vel(spas_coef)
	mushm_l:add_input(act_hm_l+uo*spas_coef)
	musbfsh_l:add_input(act_bfsh_l+uo*spas_coef)
	musgm_l:add_input(act_gm_l+uo*spas_coef)
	musip_l:add_input(act_ip_l+uo*spas_coef)
	musrf_l:add_input(act_rf_l+uo*spas_coef)
	musvi_l:add_input(act_vi_l+uo*spas_coef)
	musgs_l:add_input(act_gs_l+uo*spas_coef)
	mussl_l:add_input(act_sl_l+uo*spas_coef)
	musta_l:add_input(act_ta_l+uo*spas_coef)
end

function add_spas_ang(spas_coef)
	mushm_l:add_input(act_hm_l+uo*spas_coef)
	musbfsh_l:add_input(act_bfsh_l+uo*spas_coef)
	musgm_l:add_input(act_gm_l+uo*spas_coef)
	musip_l:add_input(act_ip_l+uo*spas_coef)
	musrf_l:add_input(act_rf_l+uo*spas_coef)
	musvi_l:add_input(act_vi_l+uo*spas_coef)
	musgs_l:add_input(act_gs_l+uo*spas_coef)
	mussl_l:add_input(act_sl_l+uo*spas_coef)
	musta_l:add_input(act_ta_l+uo*spas_coef)
end
function add_spas(spas_coef_vel, spas_coef_ang)
	
	
end
function update(model)

	-- getting current simulation time
	local t = model:time()
	-- Spasticity parameters
	local k_ang = 0.5
	local a0_flexion = 10
	local a0_extension = 80
	local L_ang = 0.2
	local L_vel = 0.2
	local vel_limit = 1
	local k = 5
	local v0_flexion = -vel_limit
	local v0_extension = vel_limit
	local slope_increase = 0.05
	local delbfsh_length = 0
	local uo = 0.1
	knee_angle = (360/(2*3.14))*math.abs(knee:position()) -- position
	knee_vel = knee:velocity()
	
	
	--if t >= 0.1 then
	--	uo = 10.0
	--end
	
	-- getting muscles length
	--local musbfsh_length = musbfsh_r:normalized_fiber_length()

	-- if type(bfsh_h) == 'table' then
		-- scone.debug('bfsh_h is table')
	-- else
		-- scone.debug('bfsh_h is not table')
	-- end
	
	--bfsh_h[n] = musbfsh_length
	time_h[n] = t
	--Sigmoid application Angle dependent
	--Sigmoid ecuation for flexion and extension (range)
	sigmoid_flexion_ang = sigmoid(k_ang * (knee_angle - a0_flexion))
	sigmoid_extension_ang = sigmoid(clip(k_ang * (knee_angle - a0_extension), -500, 500))
	spas_coef_ang = L_ang + (L_ang * (1 - (sigmoid_flexion_ang + sigmoid_extension_ang)))

	sigmoid_flexion = sigmoid(k * (knee_vel - v0_flexion))
	sigmoid_extension = sigmoid(k * (knee_vel - v0_extension))
	spas_coef_vel = L_vel + (L_vel * (1 - (sigmoid_flexion + sigmoid_extension)))
	spas_coef_vel = spas_coef_vel + slope_increase * (v0_flexion - knee_vel)
	spas_coef_vel = spas_coef_vel + slope_increase * (knee_vel - v0_extension)
	--scone.debug("spas_coef_vel: "..spas_coef_vel)
	--scone.debug("spas_coef_ang: "..spas_coef_ang)
	--scone.debug("spas_coef_vel: "..mushm_r:input())
	--hla = uo*spas_coef_ang+uo*spas_coef_vel
	--scone.debug("spas_coef_total: "..hla)
	
	hm_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	bfsh_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	gm_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	ip_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	rf_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	musvi_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	gs_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	sl_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	ta_r:add_input(uo*spas_coef_ang+uo*spas_coef_vel)
	-- scone.debug("musbfsh_length: "..musbfsh_length)
	-- scone.debug("delbfsh_length: "..delbfsh_length)
	-- scone.debug("spas_coef_vel2: "..mushm_r:activation())
	-- scone.debug("spas_coef_vel: "..hla)
	

	
	
end