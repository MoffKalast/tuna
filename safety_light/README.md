# Safety Light

A package that uses Pi kernel interfaces to periodically turn a pin high for a few miliseconds with somewhat accurate timing.

It also makes sure that the pin will be left turned off in any possible case of the node getting killed. Melting high wattage LEDs would be bad.

Blinks once on load, then waits for topic commands.

## Params

	<node name="safety_light_node" pkg="safety_light" type="safety_light_node" output="screen">
		<param name="pin" value="26" />
		<param name="on_miliseconds" value="15" />
		<param name="blink_rate" value="0.8" />
	<node>

## Subscribed Topics

 - `/safety_light` type of Bool, sets weather the light should be blinking or not, default is off