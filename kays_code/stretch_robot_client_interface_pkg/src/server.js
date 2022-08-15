var AlexaSkillServer = require("../amzn1.ask.skill.174562bb-fde8-4c01-bc34-7280e13b20c3/lambda/index.js");
AlexaSkillServer.start( {
	debug:true,
	port:2022,
	httpsPort:443,
	httpsEnabled:false,
  privateKey:'private-key.pem',
  certificate:'certificate.pem',
	// Use preRequest to load user data on each request and add it to the request json.
	// In reality, this data would come from a db or files, etc.
	preRequest: function(json,req,res) {
		console.log("preRequest fired");
		json.userDetails = { "name":"Bob Smith" };
	}
	// Add a dummy attribute to the response
	,postRequest: function(json,req,res) {
		json.dummy = "text";
	}
} )