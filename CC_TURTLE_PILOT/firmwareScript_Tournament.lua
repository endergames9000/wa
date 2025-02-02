--For Balyena SandSkiff VS+Tournament 1.18.2 (still waiting for the thruster API to be implemented in 1.20.1 )
--Watch this tutorial video on how to use this class: https://youtu.be/07Czgxqp0dk
local DroneBaseClassSP = require "lib.tilt_ships.DroneBaseClassSP"

local quaternion = require "lib.quaternions"

local instance_configs = {
	ship_constants_config = {
		DRONE_ID = ship.getId(),
		PID_SETTINGS=
		{
			POS = {
				P = 0.7,
				I = 0.001,
				D = 1.5
			},
			ROT = {
				X = {
					P = 0.04,
					I = 0.001,
					D = 0.05
				},
				Y = {
					P = 0.04,
					I = 0.001,
					D = 0.05
				},
				Z = {
					P = 0.05,
					I = 0.001,
					D = 0.05
				}
			}
		},
	},
}

local drone = DroneBaseClassSP:subclass() -- I had to make a subclass to override some functions BEFORE creating an instance (before initializing the instance)

function drone:organizeThrusterTable(thruster_table)

	local new_thruster_table = {}

	local equivalent_thrusters={
		front = { --these are the thrusters at the front of the ship
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +X				--[1]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)},	-- -X				--[2]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +Y port			--[3]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- -Y port			--[4]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +Y starboard		--[5]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- -Y starboard		--[6]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +Z				--[7]
		},
		back = {--these are the thrusters at the back of the ship
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +X				--[8]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)},	-- -X				--[9]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +Y port			--[10]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- -Y port			--[11]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- +Y starboard		--[12]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- -Y starboard		--[13]
			{thruster_count=0,total_thrust=0,equivalent_position=vector.new(0,0,0),equivalent_direction=vector.new(0,0,0)}, -- -Z				--[14]
		}
	}

	for i,thruster in pairs(thruster_table) do
		local dir = thruster.direction -- thruster facing
		local rad = thruster.radius -- position from center of mass
		local frc = thruster.base_force
		local equivalent_thruster = {}

		if(rad.z > 0) then --front of the ship

			if(dir.x>0)then--port facing thrusters
				equivalent_thruster = equivalent_thrusters.front[1]
			elseif(dir.x<0)then--starboard facing thrusters
				equivalent_thruster = equivalent_thrusters.front[2]
			end

			if(rad.x>0)then --port side
				if(dir.y>0)then -- if thruster is facing up
					equivalent_thruster = equivalent_thrusters.front[3]
				elseif(dir.y<0)then
					equivalent_thruster = equivalent_thrusters.front[4]
				end
			elseif(rad.x<=0)then--starboard side
				if(dir.y>0)then -- if thruster is facing up
					equivalent_thruster = equivalent_thrusters.front[5]
				elseif(dir.y<0)then
					equivalent_thruster = equivalent_thrusters.front[6]
				end
			end
			if(dir.z>0)then--forward thrusters
				equivalent_thruster = equivalent_thrusters.front[7]
			end
		else--back of the ship

			if(dir.x>0)then--port facing thrusters
				equivalent_thruster = equivalent_thrusters.back[1]
			elseif(dir.x<0)then--starboard facing thrusters
				equivalent_thruster = equivalent_thrusters.back[2]
			end

			if(rad.x>0)then --port side
				if(dir.y>0)then -- if thruster is facing up
					equivalent_thruster = equivalent_thrusters.back[3]
				elseif(dir.y<0)then
					equivalent_thruster = equivalent_thrusters.back[4]
				end
			elseif(rad.x<=0)then--starboard side
				if(dir.y>0)then -- if thruster is facing up
					equivalent_thruster = equivalent_thrusters.back[5]
				elseif(dir.y<0)then
					equivalent_thruster = equivalent_thrusters.back[6]
				end
			end

			if(dir.z<0)then--backward thrusters
				equivalent_thruster = equivalent_thrusters.back[7]
			end
		end
		--print(textutils.serialise({rad=rad,dir=dir}))
		--print(dir.y<0)
		equivalent_thruster.total_thrust = equivalent_thruster.total_thrust + frc
		equivalent_thruster.equivalent_position = equivalent_thruster.equivalent_position + (rad*frc)
		equivalent_thruster.thruster_count = equivalent_thruster.thruster_count + 1
		equivalent_thruster.equivalent_direction = equivalent_thruster.equivalent_direction + dir
	end

	for _,face in pairs(equivalent_thrusters) do
		for _,eqv_thruster in pairs(face) do
			eqv_thruster.equivalent_position = eqv_thruster.equivalent_position/eqv_thruster.total_thrust --gets the average position of all the thrusters weighed by each of their strengths
			eqv_thruster.equivalent_direction = (eqv_thruster.equivalent_direction/eqv_thruster.thruster_count):normalize() --gets the average direction
		end
	end
	
	local new_thruster_table_index_offset=1
	for rsi_group_name,rsi_group in pairs(equivalent_thrusters) do
		if(rsi_group_name == "front")then
			new_thruster_table_index_offset = 0
		elseif(rsi_group_name == "back")then
			new_thruster_table_index_offset = 7
		end
		for ii,eq_thruster in ipairs(rsi_group) do
			new_thruster_table[ii+new_thruster_table_index_offset]={
				radius=eq_thruster.equivalent_position,
				direction=eq_thruster.equivalent_direction,
				base_force=eq_thruster.total_thrust
			}
		end
	end

	return new_thruster_table
end

--Redstone Integrators
local RSI_TOP = peripheral.wrap("top") -- +Y:port,starboard,bow,stern
local RSI_BOTTOM = peripheral.wrap("bottom") -- -Y:port,starboard,bow,stern
local RSI_FRONT = peripheral.wrap("front") -- +Z, +X, -X
local RSI_BACK = peripheral.wrap("back") -- -Z, +X, -X

function drone:powerThrusters(component_redstone_power)
	if(type(component_redstone_power) == "number")then
		--these are the thrusters at the FRONT of the ship
		RSI_FRONT.setAnalogOutput("east", component_redstone_power)			-- +X				--[1]
		RSI_FRONT.setAnalogOutput("west", component_redstone_power)			-- -X				--[2]
		RSI_TOP.setAnalogOutput("south", component_redstone_power)			-- +Y port			--[3]
		RSI_BOTTOM.setAnalogOutput("south", component_redstone_power)		-- -Y port			--[4]
		RSI_TOP.setAnalogOutput("west", component_redstone_power)			-- +Y starboard		--[5]
		RSI_BOTTOM.setAnalogOutput("west", component_redstone_power)		-- -Y starboard		--[6]
		RSI_FRONT.setAnalogOutput("south", component_redstone_power)		-- +Z				--[7]

		--these are the thrusters at the BACK of the ship
		RSI_BACK.setAnalogOutput("east", component_redstone_power)			-- +X				--[8]
		RSI_BACK.setAnalogOutput("west", component_redstone_power)			-- -X				--[9]
		RSI_TOP.setAnalogOutput("east", component_redstone_power)			-- +Y port			--[10]
		RSI_BOTTOM.setAnalogOutput("east", component_redstone_power)		-- -Y port			--[11]
		RSI_TOP.setAnalogOutput("north", component_redstone_power)			-- +Y starboard		--[12]
		RSI_BOTTOM.setAnalogOutput("north", component_redstone_power)		-- -Y starboard		--[13]
		RSI_BACK.setAnalogOutput("north", component_redstone_power)			-- -Z				--[14]
	else
		--these are the thrusters at the FRONT of the ship
		RSI_FRONT.setAnalogOutput("east", component_redstone_power[1])		-- +X				--[1]
		RSI_FRONT.setAnalogOutput("west", component_redstone_power[2])		-- -X				--[2]
		RSI_TOP.setAnalogOutput("south", component_redstone_power[3])		-- +Y port			--[3]
		RSI_BOTTOM.setAnalogOutput("south", component_redstone_power[4])	-- -Y port			--[4]
		RSI_TOP.setAnalogOutput("west", component_redstone_power[5])		-- +Y starboard		--[5]
		RSI_BOTTOM.setAnalogOutput("west", component_redstone_power[6])		-- -Y starboard		--[6]
		RSI_FRONT.setAnalogOutput("south", component_redstone_power[7])		-- +Z				--[7]

		--these are the thrusters at the BACK of the ship
		RSI_BACK.setAnalogOutput("east", component_redstone_power[8])		-- +X				--[8]
		RSI_BACK.setAnalogOutput("west", component_redstone_power[9])		-- -X				--[9]
		RSI_TOP.setAnalogOutput("east", component_redstone_power[10])		-- +Y port			--[10]
		RSI_BOTTOM.setAnalogOutput("east", component_redstone_power[11])	-- -Y port			--[11]
		RSI_TOP.setAnalogOutput("north", component_redstone_power[12])		-- +Y starboard		--[12]
		RSI_BOTTOM.setAnalogOutput("north", component_redstone_power[13])	-- -Y starboard		--[13]
		RSI_BACK.setAnalogOutput("north", component_redstone_power[14])		-- -Z				--[14]
		-- --these are the thrusters at the FRONT of the ship
		-- RSI_FRONT.setAnalogOutput("east", 0)	-- +X				--[1]
		-- RSI_FRONT.setAnalogOutput("west", 0)	-- -X				--[2]
		-- RSI_TOP.setAnalogOutput("south", 0)	-- +Y port			--[3]
		-- RSI_BOTTOM.setAnalogOutput("south", 0)-- -Y port			--[4]
		-- RSI_TOP.setAnalogOutput("west", 0)	-- +Y starboard		--[5]
		-- RSI_BOTTOM.setAnalogOutput("west", 0)-- -Y starboard		--[6]
		-- RSI_FRONT.setAnalogOutput("south", 0)	-- +Z				--[7]

		-- --these are the thrusters at the BACK of the ship
		-- RSI_BACK.setAnalogOutput("east", 0)	-- +X				--[8]
		-- RSI_BACK.setAnalogOutput("west", 0)	-- -X				--[9]
		-- RSI_TOP.setAnalogOutput("east", 0)	-- +Y port			--[10]
		-- RSI_BOTTOM.setAnalogOutput("east", 0)-- -Y port			--[11]
		-- RSI_TOP.setAnalogOutput("north", 0)	-- +Y starboard		--[12]
		-- RSI_BOTTOM.setAnalogOutput("north", 0)-- -Y starboard		--[13]
		-- RSI_BACK.setAnalogOutput("north", 15)	-- -Z				--[14]
	end
end

-- Watch this tutorial video to learn how to use this function: https://youtu.be/07Czgxqp0dk?si=gltpueMIgFjHpqJZ&t=269 (skip to 4:29)
function drone:getOffsetDefaultShipOrientation(default_ship_orientation)
	return quaternion.fromRotation(default_ship_orientation:localPositiveY(), -90)*default_ship_orientation -- Rotates the default orientation so that the nose of the ship is aligned with it's local +X axis
end

-- Watch this tutorial video to learn how to use this function: https://youtu.be/07Czgxqp0dk?si=gltpueMIgFjHpqJZ&t=269 (5:25 to 10:30)
function drone:customFlightLoopBehavior(customFlightVariables)
	self.target_rotation = quaternion.fromToRotation(self.target_rotation:localPositiveX(),vector.new(0,0,1))*self.target_rotation
	--self.target_rotation = quaternion.fromToRotation(self.target_rotation:localPositiveY(),vector.new(0,-1,0))*self.target_rotation -- uncomment to flip ship upside down
	self.target_global_position = vector.new(X,Y,Z) --replace XYZ with world coordinates
end

local customDrone = drone(instance_configs)

customDrone:run()
