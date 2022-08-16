/* *
 * This sample demonstrates handling intents from an Alexa skill using the Alexa Skills Kit SDK (v2).
 * Please visit https://alexa.design/cookbook for additional examples on implementing slots, dialog management,
 * session persistence, api calls, and more.
 * */
const Alexa = require('ask-sdk-core');
const skillName = 'Stretch Skills';
const persistenceAdapter = require('ask-sdk-s3-persistence-adapter');

// Connecting to ROS
const ROSLIB = require('roslib');

// rosbridge_websocket to port 2022 on Kay's desktop in Duckie Town
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:2022'
});

ros.on('connection', function() {
    console.log('publish-example: Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('publish-example: Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('publish-example: Connection to websocket server closed.');
});

var command_topic = new ROSLIB.Topic({
    ros: ros, 
    name: '/client_command', 
    messageType: 'std_msgs/String'
});//to inform user alexa was used

const LaunchRequestHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'LaunchRequest';
    },
    handle(handlerInput) {
        const speakOutput = handlerInput.t('WELCOME_MSG');

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const HelloWorldIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'HelloWorldIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'Hello World!';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const DrivingIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'DrivingIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.direction.value;
        var speakOutput = 'Driving!';
        var str = new ROSLIB.Message({
            data: 'driving'
        });
        
        if (slot === 'forward' || slot === 'forwards' || slot === 'ahead')
            speakOutput = 'Driving Forwards!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'driving base forward'
            });
            command_topic.publish(str);
            
        if (slot === 'back' || slot === 'backwards' || slot === 'backward')
            speakOutput = 'Driving Backwards!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'driving base backwards'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const RotatingBaseIndentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'RotatingBaseIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.side.value;
        var speakOutput = 'Rotating Base!';
        var str = new ROSLIB.Message({
            data: 'rotating base'
        });
        
        if (slot === 'right' || slot === 'clockwise')
            speakOutput = 'Rotating Base to the Right!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'turn base right'
            });
            command_topic.publish(str);
            
        if (slot === 'left' || slot === 'counterclockwise')
            speakOutput = 'Rotating Base to the Left!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'turn base left'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const LiftIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'LiftIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.move.value;
        var speakOutput = 'Moving Lift!';
        var str = new ROSLIB.Message({
            data: 'moving lift'
        });
        
        if (slot === 'up' || slot === 'lift' || slot === 'raise')
            speakOutput = 'Raising Lift!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'move lift upwards'
            });
            command_topic.publish(str);
            
        if (slot === 'down' || slot === 'lower')
            speakOutput = 'Lowering Lift!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'move lift downwards'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const ArmIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'ArmIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.size.value;
        var speakOutput = 'Changing Arm Length!';
        var str = new ROSLIB.Message({
            data: 'moving arm'
        });
        
        if (slot === 'out' || slot === 'extend')
            speakOutput = 'Extending Arm!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'make arm extend'
            });
            command_topic.publish(str);
            
        if (slot === 'in' || slot === 'collapse')
            speakOutput = 'Collapsing Arm!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'make arm collapse'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const GripperIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'GripperIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.pinch.value;
        var speakOutput = 'Adjusting Gripper!';
        var str = new ROSLIB.Message({
            data: 'moving gripper'
        });
        
        if (slot === 'close' || slot === 'in' || slot === 'together')
            speakOutput = 'Close Gripper!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'make gripper close'
            });
            command_topic.publish(str);
            
        if (slot === 'open' || slot === 'apart' || slot === 'out')
            speakOutput = 'Opening Gripper!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'make gripper open'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const WristIntentHandler ={
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'WristIntent';
    },
    handle(handlerInput) {
        var slot = handlerInput.requestEnvelope.request.intent.slots.turn.value;
        var speakOutput = 'Rotating Wrist!';
        var str = new ROSLIB.Message({
            data: 'rotating wrist'
        });
        
        if (slot === 'right' || slot === 'clockwise')
            speakOutput = 'Rotating Wrist to the Right!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'wrist rotating right'
            });
            command_topic.publish(str);
            
        if (slot === 'left' || slot === 'counterclockwise')
            speakOutput = 'Rotating Wrist to the Left!';
            command_topic.advertise();
            str = ROSLIB.Message({
                data : 'wrist rotating left'
            });
            command_topic.publish(str);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

const HelpIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.HelpIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'You can say hello to me! How can I help?';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const CancelAndStopIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && (Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.CancelIntent'
                || Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.StopIntent');
    },
    handle(handlerInput) {
        const speakOutput = 'Stopped!';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.getResponse();
            .getResponse();
    }
};
/* *
 * FallbackIntent triggers when a customer says something that doesn’t map to any intents in your skill
 * It must also be defined in the language model (if the locale supports it)
 * This handler can be safely added but will be ingnored in locales that do not support it yet 
 * */
const FallbackIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.FallbackIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'Sorry, I don\'t know about that. Please try again.';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};
/* *
 * SessionEndedRequest notifies that a session was ended. This handler will be triggered when a currently open 
 * session is closed for one of the following reasons: 1) The user says "exit" or "quit". 2) The user does not 
 * respond or says something that does not match an intent defined in your voice model. 3) An error occurs 
 * */
const SessionEndedRequestHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'SessionEndedRequest';
    },
    handle(handlerInput) {
        console.log(`~~~~ Session ended: ${JSON.stringify(handlerInput.requestEnvelope)}`);
        // Any cleanup logic goes here.
        return handlerInput.responseBuilder.getResponse(); // notice we send an empty response
    }
};
/* *
 * The intent reflector is used for interaction model testing and debugging.
 * It will simply repeat the intent the user said. You can create custom handlers for your intents 
 * by defining them above, then also adding them to the request handler chain below 
 * */
const IntentReflectorHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest';
    },
    handle(handlerInput) {
        const intentName = Alexa.getIntentName(handlerInput.requestEnvelope);
        const speakOutput = `You just triggered ${intentName}`;

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};
/**
 * Generic error handling to capture any syntax or routing errors. If you receive an error
 * stating the request handler chain is not found, you have not implemented a handler for
 * the intent being invoked or included it in the skill builder below 
 * */
const ErrorHandler = {
    canHandle() {
        return true;
    },
    handle(handlerInput, error) {
        const speakOutput = 'Sorry, I had trouble doing what you asked. Please try again.';
        console.log(`~~~~ Error handled: ${JSON.stringify(error)}`);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

/**
 * This handler acts as the entry point for your skill, routing all request and response
 * payloads to the handlers above. Make sure any new handlers or interceptors you've
 * defined are included below. The order matters - they're processed top to bottom 
 * */
exports.handler = Alexa.SkillBuilders.custom()
    .addRequestHandlers(
        LaunchRequestHandler,
        HelloWorldIntentHandler,
        DrivingIntentHandler,
        RotatingBaseIndentHandler,
        LiftIntentHandler,
        ArmIntentHandler,
        GripperIntentHandler,
        WristIntentHandler,
        HelpIntentHandler,
        CancelAndStopIntentHandler,
        FallbackIntentHandler,
        SessionEndedRequestHandler,
        IntentReflectorHandler
        )
    .addErrorHandlers(
        ErrorHandler)
    .withPersistenceAdapter(
        new persistenceAdapter.S3PersistenceAdapter({bucketName:process.env.S3_PERSISTENCE_BUCKET}))
    .withApiClient(new Alexa.DefaultApiClient())
    .lambda();