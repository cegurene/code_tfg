function init(model, par)

	rf_r	= model:find_actuator("rect_fem_r")
	scone.debug("rf_r: "..rf_r:name())
	vi_r 	= model:find_actuator("vas_int_r")
	scone.debug("vi_r: "..vi_r:name())
	ta_r	= model:find_actuator("tib_ant_r")
	scone.debug("ta_r: "..ta_r:name())
	
	bfsh_r	= model:find_actuator("bifemsh_r")
	scone.debug("bfsh_r: "..bfsh_r:name())
	musbfsh_r	= model:find_muscle("bifemsh_r")
	scone.debug("musbfsh_r: "..musbfsh_r:name())
	
	O_rf_r		= par:create_from_mean_std(rf_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_vi_r		= par:create_from_mean_std(vi_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_ta_r		= par:create_from_mean_std(ta_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	O_bfsh_r	= par:create_from_mean_std(bfsh_r:name() .. "-offset", 0.015, 0.1, 0.015, 0.05)
	
	
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
	
end

function update(model)

	-- getting current simulation time
	local t = model:time()
	local delbfsh_length = 0

	if t >= 1.0 then
		uo = 1.0
	end
	
	-- getting muscles length
	local musbfsh_length = musbfsh_r:normalized_fiber_length()

	-- if type(bfsh_h) == 'table' then
		-- scone.debug('bfsh_h is table')
	-- else
		-- scone.debug('bfsh_h is not table')
	-- end
	
	bfsh_h[n] = musbfsh_length
	time_h[n] = t
	
	for i = 1, n do
		if time_h[i] <= (t-delay) then
			
			delbfsh_length = bfsh_h[i]
			break
		end
	end
	for i = 1, n-1 do
		time_h[i] = time_h[i+1]
		bfsh_h[i] = bfsh_h[i+1]
		
	end
	
	-- scone.debug("musbfsh_length: "..musbfsh_length)
	-- scone.debug("delbfsh_length: "..delbfsh_length)
	
	rf_r:add_input(O_rf_r + uo*act1)
	vi_r:add_input(O_vi_r + uo*act1)
	ta_r:add_input(O_ta_r + uo*act2)
	
	bfsh_r:add_input(O_bfsh_r + gl1*(delbfsh_length-Lo_bfsh_r))

	
end