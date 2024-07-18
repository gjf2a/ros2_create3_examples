import speech_recognition as sr
import os 
import delivery_plan_runner_pi5_base as HTN
import requests
import json
import re
import gtts
import sys
import pickle
import sounddevice
from playsound import playsound  

class LLMConnector: #class used to create calls to Ollama API
    def __init__(self, LLM, systemMessage): #takes the name and a system message as input and creates the dictionary needed to access the llm
        self.url = "http://localhost:11434/api/generate" #local ollama port
        self.headers = {
            "Content-Type": "application/json"
        }
        self.data = {
            "model": LLM,
            "prompt": "",
            "stream": False,
            "system": systemMessage
        }

    def prompt(self, promptMessage): #takes prompt as input, updates dictionary, and creates call to llm through ollama
        self.data["prompt"] = promptMessage 
        response = requests.post(self.url, headers=self.headers, data=json.dumps(self.data)) #posts request to ollama API and recieves response
        if response.status_code == 200: #code for success
            response_text = response.text
            response = json.loads(response.text)["response"] #extracts response from json object
            print("Response: " + response) # prints llm response for testing purposes
            return response
        else:
            print( "API Error:", response.status_code, response.text)
            return None

def outputSpeech(text): #method for text to voice output, takes message as input
   # tempSound = gtts.gTTS(text) #creates voice recording
   # tempSound.save("tempFile.mp3") #saves recording as local file
   # playsound("tempFile.mp3") #outputs file as sound
   # os.remove("tempFile.mp3") #deletes local file
    print(text)
      
def getSpeechInput(output): #outputs message, returns result of voice input, takes message as input
       # outputSpeech(output) #calls outputSpeech to give prompt
       # r = sr.Recognizer() #creates instance on speech recognition object
       # mic = sr.Microphone() #creates object for microphone
       # try:
       #     with mic as source: #sets microphone as source for speech input
       #         audio = r.listen(source) #gets audio from input source and saves as variable
       #     input = r.recognize_wit(audio, key="HGEODKAPMSH73UNQHATKFVWJZUZYKFUZ").lower() #gets transcription from wit.ai (meta) API and puts in lower case
       #     print("Input: " + input) #prints recognized speech for testing purposes
       #     return input
       # except: #error typically occurs from no input
       #     return getSpeechInput("waiting for input") #tries again
    return input(output)
def getStateDescription(runner):
    description = "The following is a description of the destinations the robot can travel to " + runner.state.description
    if len(runner.state.package_locations) > 0:
        description += " The following is a description of where each package is located. "
        for package in runner.state.package_locations:
            description += package + " is located at " + runner.state.package_locations[package] + ". "
    else:
        description += " There are no packages in the system."
    return description

class PackageDeliveryState(): #state in which robot delivers package from current location to a designated destination
    def __init__(self, runner): #initiated with runner
        self.runner = runner
        #Sets up instance of object that is used to generate method calls through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        message = getStateDescription(self.runner) + " Based on the following input, you are to identify a package and the destination the user would like the package to be delivered to. Output your findings in the following format **package** **destination**, replacing **package** with the name of the package and **destination** with the name of the destination, as identified from the input. Output only a single room and package in the specifed format with no extra characters, instructions, explanations, or labels."
        self.methodCaller = LLMConnector("phi3:3.8b", message)
        #Sets up instance of object that is used to evaluate user verification through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        self.classifier = LLMConnector("phi3:3.8b", "You are an expert classifier who determines if the prompt is a positive or negative response. If it is a positive response, output a 1. If it is a negative response or you are unsure, output a 0. Do not include any additional text, explanations, or notes.")
    
    def action(self): #action prompts for instructions and generates plan
        prompt = getSpeechInput("Please provide an item and desination.") #gets item and destination from user through speech input
        for i in range(5): #allows four additional attempts to fine tune prompt before failing process
            deliveryDetails = self.methodCaller.prompt(prompt) #recieves details in specifed format from llm
            deliveryMethod = "deliver " + deliveryDetails #puts details of delivery into method call
            parts = deliveryDetails.split() #seperates location and item
            if len(parts) == 2 and parts[0] in self.runner.state.package_locations and parts[1] in self.runner.state.graph: #verifies that method call contains valid package and location
                response = getSpeechInput("To confirm, would you like " + parts[0] + " to be delivered to " + parts[1] + "?") #verifies request using package and location
                classification = self.classifier.prompt(response) #recieves a 0 or 1 as a response from llm- 1 indicates positive verification
                if '1' in classification: #user has verified method call
                    self.runner.current_input = deliveryMethod #sets method as runner's current input
                    self.runner.deliver() #executes method call
                    break #breaks loop because no further fine tuning is needed
                elif i == 4: #user has not verified prompt and all attempts are used
                    outputSpeech("Unable to verify instructions")
                else: #user has not verified prompt, but attempts remain
                    newInfo = getSpeechInput("Please clarify your request") #gets clarification from user for fine tuning
                    prompt = prompt + newInfo #adds additional instructions to original prompt
            else: #tries again with same prompt because the response from the llm was bad
                outputSpeech("Trying again")
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class DescriptionState(): #state in which llm provides description of state of system
    def __init__(self, runner): #initialized with runner
        self.runner = runner
        message = getStateDescription(self.runner) + "Based only on the provided information, breifly describe where each package is currently located in plain english. Do not provide additional explanations or speculation. Do not make up or describe any packages that are not explicitly listed with a location in the description." 
        #creates instance of LLM Connector that sets up model to recieve a list of current locations and describe the sytem
        self.describer = LLMConnector("phi3:instruct", message)
    def action(self): #action outputs a description of the state of the system
        outputSpeech(self.describer.prompt("Provide a short description.")) #creates call to llm with all locations and outputs resulting description
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class QuestionState(): #state in which the user can ask a question for clarification
    def __init__(self, runner): #initialized with state and planner
        self.runner = runner
        message = "You are part of an artificial intelligence system that controls the movement of an iRobot Create3 robot. The robot can be navigated between any two destinations and deliver packages. " + getStateDescription(self.runner) + " The following is a question asked by the user. Do your best to provide a breif response based on the previous information. "
        #creates local instance of connector that describes the premise of the system and sets the llm up to recieve a list of locations and a question to answer
        self.answerer = LLMConnector("phi3:instruct", message)
    def action(self): #action gets question from user and outputs response
        question = getSpeechInput("What is your question?") #asks user for question and saves it
        outputSpeech(self.answerer.prompt("Question to be answered: " + question)) #prompts and gets response from llm, outputs reponse
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class NavigationState(): #state in which the robot moves from current location to new location
    def __init__(self, runner): #initialized with state and runner
        self.runner = runner
        message = getStateDescription(self.runner) + " Based on the following input, you are to identify the name of the destination the user would like the robot to navigate to and output it in the following format **destination** by replacing **destination** with the one identified. Output only the name of a single destination with no extra characters, instructions, explanations, or labels."
        #Sets up instance of object that is used to generate method calls through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        self.methodCaller = LLMConnector("phi3:3.8b", message)
        #Sets up instance of object that is used to evaluate user verification through ollama API with phi3:3.8b as the model 
        self.classifier = LLMConnector("phi3:3.8b", "You are an expert classifier who determines if the prompt is a positive or negative response. If it is a positive response, output a 1. If it is a negative response or you are unsure, output a 0. Do not include any additional text, explanations, or notes.")
    
    def action(self): #action gets location from user and generates plan for robot to travel to location
        prompt = getSpeechInput("Please provide a desination.") #gets item and destination from user through speech input
        for i in range(5): #allows four additional attempts to fine tune prompt before failing process
            navigationDetails = self.methodCaller.prompt(prompt) #recieves details in specifed format from llm
            navigationMethod = "go " + navigationDetails #puts details of navigation into method call
            parts = navigationDetails.split() #seperates location and item
            if len(parts) == 1 and parts[0] in self.runner.state.graph: #verifies that method call contains valid location
                response = getSpeechInput("To confirm, would you like the robot to navigate to" + parts[0] + "?") #verifies request using location
                classification = self.classifier.prompt(response) #recieves a 0 or 1 as a response from llm- 1 indicates positive verification
                if '1' in classification: #user has verified method call
                    self.runner.current_input = navigationMethod #sets method as runner's current input
                    self.runner.go() #executes method call
                    break #breaks loop because no further fine tuning is needed
                elif i == 4: #user has not verified prompt and all attempts are used
                    outputSpeech("Unable to verify instructions")
                else: #user has not verified prompt, but attempts remain
                    newInfo = getSpeechInput("Please clarify your request") #gets clarification from user for fine tuning
                    prompt = prompt + newInfo #adds additional instructions to original prompt
            elif i<4: #tries again with same prompt because the response from the llm was bad
                outputSpeech("Trying again")
            else:
                outputSpeech("Process Failed")
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class RoutingState(): #state in which the system determines which state the user would like to access
    def __init__(self, runner): #takes runner to initialize- not used directly, but necessary for consitency of state machine and to pass to next state
        self.runner = runner
        #creates instance of llm connector that includes a description of each state and sets the llm up to determine which state the user would like to access
        self.classifier = LLMConnector("phi3:3.8b", """You are part of a larger system that can 
                                       (0) deliver a package
                                       (1) navigate a robot
                                       (2) describe the current state of the system
                                       (3) answer questions regarding the system
                                       (4) quit the program
                                       Based on the following input, determine which of the abilities the user would like to access and output only the corresponding number, no text. 
                                       """)
    def action(self): #action takes input from user and returns the desired state
        while True: #in loop so that it will try to determine the appropriate state again if the process fails
            request = getSpeechInput("Would you like to deliver a package, navigate the robot, get a description of the system, ask a question, or quit the program?") #gives user options and recieves reponse
            classification = self.classifier.prompt(request) #prompts llm with user input
            try: 
                num = int(classification) #tries to cast llm response to integer
                if num == 0: #0 means the user wants to deliver a package
                    return PackageDeliveryState(self.runner) #returns next state which is package delivery
                elif num == 1: #1 means the user wants to navigate the robot
                    return NavigationState(self.runner) #returns next state which is navigation
                elif num == 2: #2 means the user want a description of the system
                    return DescriptionState(self.runner) #returns next state which is description
                elif num == 3: #3 means the user wants to ask a question 
                    return QuestionState(self.runner) #returns next state which is question
                elif num == 4: 
                    return -1
                else: outputSpeech("Please try again.")
            except: #catches any errors in case the response is in an improper format
                outputSpeech("Please try again.")

def main(args=None):
    with open(sys.argv[2] + "/" + sys.argv[2] + "_description") as f:
        map_description = f.read()
    with open(sys.argv[2] + "/" + sys.argv[2] + "_map", 'rb') as f:
        map_data = pickle.load(f)
    runner = HTN.RobotMapRunner(sys.argv[2], sys.argv[1], map_data, map_description)
    runner.st.start()
    runner.ht.start()
    runner.cmd_queue.put('reset')
    systemState = RoutingState(runner) #sets first state of state machine to routing
   # os.system("ollama run phi3:3.8b /bye") #ensures ollama is open locally
    with open(sys.argv[2] + "/" + sys.argv[2] + "_packages") as f:
        for package in f:
            runner.current_input = "at " + package
            runner.at()
    while systemState != -1: #continously runs actions of state and gets next state
        systemState = systemState.action()
    print("test")
    runner.finished.set()
    runner.ht.join()
    runner.st.join()
    outputSpeech("Bye")
    sys.quit()
if __name__ == '__main__':
    main()
