// Global variables
let recognition;
let outputDiv;
let command="";

// Initialize the Web Speech API
function initialize() {

  recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
  recognition.interimResults = true;
  recognition.continuous = true;
  recognition.lang = 'en-US';

  recognition.onresult = function (event) {
    let interimTranscript = '';
    for (let i = event.resultIndex; i < event.results.length; ++i) {
      if (event.results[i].isFinal) {
        // Append final transcript to output div
        outputDiv.innerHTML += '<p>' + event.results[i][0].transcript + '</p>';
      } else {
        // Append interim transcript to output div
        interimTranscript += event.results[i][0].transcript;
      }
    }


    // Display interim transcript in output div
    outputDiv.innerHTML = interimTranscript;
  };

  recognition.onerror = function (event) {
    console.error('Recognition error:', event.error);
  };
}

// Start recording
function startRecording() {
  outputDiv = document.getElementById('output');
  outputDiv.innerHTML = '';
  recognition.start();
}

// Stop recording
function stopRecording() {
  recognition.stop();
  command = outputDiv.innerHTML;
  console.log('Text command:', command);
}


// Speak the given text
function speak(text) {
  let utterance = new SpeechSynthesisUtterance(text);
  // Initialize the TTS engine
  let tts_engine = new SpeechSynthesisUtterance();
  tts_engine.text = text //"Message received. Sending to ChatGPT.";
  window.speechSynthesis.speak(tts_engine);

  //tts_engine.speak(utterance.text);
}

// Send to robot
async function sendToRobot() {
  //let command = outputDiv.innerHTML;
  console.log("Command to be sent: "+ command);

  // Send the voice command to the robot for further processing
  try {
    let response = await fetch("http://localhost:5000/rosgpt", {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded"
      },
     
      body: new URLSearchParams({
        text_command: command
      }),
    });

    let jsonResponse = await response.json();
    console.log("Received response from robot:", jsonResponse);
    outputDiv.innerHTML += '<p>Robot response: ' + jsonResponse.response + '</p>';
    speak("Robot response: " + jsonResponse.response);
  } catch (error) {
    console.error("Error sending command to robot:", error);
  }
}

// Initialize the app
window.onload = function () {
  initialize();

  // Add event listeners to buttons
  document.getElementById('startBtn').addEventListener('click', startRecording);
  document.getElementById('stopBtn').addEventListener('click', stopRecording);
  document.getElementById('sendBtn').addEventListener('click', sendToRobot);
};
