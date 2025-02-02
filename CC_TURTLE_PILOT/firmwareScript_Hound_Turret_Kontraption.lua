--For Hound Turret using VS+Kontraption
local DroneBaseClassKontraption = require "lib.tilt_ships.DroneBaseClassKontraption"
local HoundTurretArsNouveauMagitech = require "lib.tilt_ships.HoundTurretArsNouveauMagitech"
local HoundTurretArsNouveauMagitech1201 = require "lib.tilt_ships.HoundTurretArsNouveauMagitech1201"

local quaternion = require "lib.quaternions"


local instance_configs = {
	radar_config = {
		designated_ship_id = "267",
		designated_player_name="PHO", --replace this with your username
		ship_id_whitelist={},
		player_name_whitelist={},
	},
	ship_constants_config = {
		DRONE_ID = ship.getId(),
		PID_SETTINGS=
		{
			POS = {
				P=0.2,
				I=0.001,
				D=0.25,
			},
		},
		ION_THRUSTERS_COUNT = { --number of thrusters pointing in each cardinal direction
        	pos=vector.new(2,2,2), 	-- +X, +Y, +Z
        	neg=vector.new(2,2,2)	-- -X, -Y, -Z
    	}
	},
	channels_config = {
		DEBUG_TO_DRONE_CHANNEL = 9,
		DRONE_TO_DEBUG_CHANNEL = 10,
		
		REMOTE_TO_DRONE_CHANNEL = 7,
		DRONE_TO_REMOTE_CHANNEL = 8,
		
		DRONE_TO_COMPONENT_BROADCAST_CHANNEL = 800,
		COMPONENT_TO_DRONE_CHANNEL = 801,
		
		EXTERNAL_AIM_TARGETING_CHANNEL = 1009,
		EXTERNAL_ORBIT_TARGETING_CHANNEL = 1010,
		EXTERNAL_GOGGLE_PORT_CHANNEL = 1011,
		REPLY_DUMP_CHANNEL = 10000,
	},
	rc_variables = {
		
	},
}

local drone = HoundTurretArsNouveauMagitech:subclass() -- I had to make a subclass to override some functions BEFORE creating an instance (before initializing the instance)

if(string.find(_HOST,"Minecraft 1.20.1")) then
	drone = HoundTurretArsNouveauMagitech1201:subclass() -- Hound Turrets work differently in 1.20.1. It uses hexcasting to speedup the spell bullets
end

function drone:setShipFrameClass(configs) --override this to set ShipFrame Template
	self.ShipFrame = DroneBaseClassKontraption(configs)
end

function drone:alternateFire(step)
	local seq_1 = step==0
	local seq_2 = step==1
	--{modem_block, redstoneIntegrator_side}
	self:activateAllGuns({"top","right"},seq_1)
	self:activateAllGuns({"top","back"},seq_1)
	self:activateAllGuns({"top","top"},seq_2)
end

local customDrone = drone(instance_configs)

customDrone:run()
