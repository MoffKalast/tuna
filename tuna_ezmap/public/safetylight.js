var safetylight;

class SafetyLight {

	constructor(){

		this.light_state = false;

		this.light_topic = new ROSLIB.Topic({
			ros : ros,
			name : 'safety_light',
			messageType : 'std_msgs/Bool'
		});

		this.light_topic.subscribe(function(msg) {
			if(msg.data == safetylight.light_state)
				return

			safetylight.light_state = msg.data;

			if(safetylight.light_state){
				document.getElementById("lsafetylighticon").src = "assets/img/light_on.svg";
				document.getElementById("psafetylighticon").src = "assets/img/light_on.svg";
			}else{
				document.getElementById("lsafetylighticon").src = "assets/img/light_off.svg";
				document.getElementById("psafetylighticon").src = "assets/img/light_off.svg";
			}
			
		});
	}

	static toggle(){
		safetylight.light_topic.publish(new ROSLIB.Message({
			data: !safetylight.light_state
		}));
	}
}

window.addEventListener('load', function() {
	safetylight = new SafetyLight();
});
