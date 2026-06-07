function init(model)
	knee = model:find_dof("knee_rz_r")
	knee_angle = math.abs(knee:position())
end

function update(model)
	knee_angle = knee:position()
	knee_angle = math.abs(knee_angle)
	-- scone.debug(knee_angle)
	return false
end

function result(model)

	return knee_angle*180/math.pi
end

function store_data(frame)
	frame:set_value("knee_angle", knee_angle)
end