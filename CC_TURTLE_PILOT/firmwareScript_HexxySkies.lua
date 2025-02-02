--For HexxySkies and HexTweaks 4.0.0+
local DroneBaseClassHexxySkies = require "lib.tilt_ships.DroneBaseClassHexxySkies"

local quaternion = require "lib.quaternions"

local instance_configs = {}

local drone = DroneBaseClassHexxySkies(instance_configs)

--[[
-- Watch this tutorial video to learn how to use this function: https://youtu.be/07Czgxqp0dk?si=gltpueMIgFjHpqJZ&t=269 (skip to 4:29)
function drone:getOffsetDefaultShipOrientation(default_ship_orientation)
	return quaternion.fromRotation(default_ship_orientation:localPositiveY(), -90)*default_ship_orientation -- Rotates the default orientation so that the nose of the ship is aligned with it's local +X axis
end
]]--

-- Watch this tutorial video to learn how to use this function: https://youtu.be/07Czgxqp0dk?si=gltpueMIgFjHpqJZ&t=269 (5:25 to 10:30)
function drone:customFlightLoopBehavior(customFlightVariables)
	self.target_rotation = quaternion.fromToRotation(self.target_rotation:localPositiveX(),vector.new(0,0,1))*self.target_rotation
	--self.target_rotation = quaternion.fromToRotation(self.target_rotation:localPositiveY(),vector.new(0,-1,0))*self.target_rotation -- uncomment to flip ship upside down
	self.target_global_position = vector.new(268,135,159) --replace XYZ with world coordinates
end

drone:run()
