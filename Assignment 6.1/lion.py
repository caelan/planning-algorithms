file = open('lion.mdp', 'w')

def write(values):
	string = ""
	for (i, value) in enumerate(values):
		if i == len(values)-1:
			string += (str(value) + "\n")
		else:
			string += (str(value) + " ")
	file.write(string)

resolution = 1.0
capacity = 30.0
zebra_mass = 164.0
hunt_energy = .5
day_energy = 6.0

hunt_actions = {1: .15, 2: .33, 3: .37, 4: .40, 5: .42, 6: .43}

write(['DEAD', -1.0, 'Terminal'])
states = [] #And dead State
state = 0.0
while state <= capacity:
	states.append(state)
	write([state, 0.0])
	state += resolution
write([states[-1]]) #Initial State

def closest_state(value):
	closest = 'DEAD'
	for state in states:
		if state <= value:
			closest = state
		else:
			return closest
	return closest

for state in states:
	write([state, 'NO_HUNT', closest_state(state - day_energy), 1])
	for (action, prob) in hunt_actions.items():
		write([state, action, closest_state(state + zebra_mass/action - hunt_energy - day_energy), prob, closest_state(state - hunt_energy - day_energy), 1-prob])

file.close()
