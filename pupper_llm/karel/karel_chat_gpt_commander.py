import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from openai import OpenAI
import karel  # Importing your KarelPupper API

client = OpenAI(api_key='')

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',  # Replace with your topic name for responses
            10
        )

        self.get_logger().info('GPT-4o conversation node started and waiting for queries...')

        # Initialize the text-to-speech engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Set the speed of speech (optional)

        # Initialize KarelPupper robot control
        self.pupper = karel.KarelPupper()

    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        pass
        # Paste in your implementation from simple_gpt_chat.py
         def query_callback(self, msg):
        # Extract the user query from the message using the data attribute of message
        user_query = msg.data
        # Call GPT-4o API to get the response. Use the get_gpt4_response method and pass in the query
        response = self.get_gpt4_response(user_query)
        # Publish the response (as the data to a String message) using self.publisher_ and its publish method, 
        response_msg = String()
        response_msg.data = response
        self.publisher_.publish(response_msg)
        # Play the response through the speaker with the play_response method
        self.play_response(response)
        # Parse and execute robot commands if present with the execute_robot_command method
        self.execute_robot_command(response)

    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4o using OpenAI's Python client
            prompt = "You are a robot assistant for a small quadruped robot called Pupper. Based on the user’s natural language input, respond with a single command that the robot can understand. The command should be one of: 'walk forward', 'walk backward', 'turn left', 'turn right', 'sit', 'stand', or 'stop'. Only return the command in lowercase."
            self.get_logger().info(f"Initial Prompt: {prompt}")
            response = client.chat.completions.create(model="gpt-4o",  # Model identifier, assuming GPT-4o is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            max_tokens=150)  # Adjust token limit based on your requirement

            # Extract the assistant's reply from the response
            gpt4_response = response.choices[0].message.content
            return gpt4_response

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4o API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

    def play_response(self, response):
        try:
            # Use the TTS engine to say the response out loud
            self.tts_engine.say(response)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Error playing response through speaker: {str(e)}")

    def execute_robot_command(self, response):
        # Convert the response to lowercase to handle case-insensitivity
        response = response.lower()
        self.get_logger().info(f"Response: {response}")
        # TODO: Implement the robot command execution logic, in a large if-else statement. Your conditionals should be set based on the expected commands from GPT-4o, and the corresponding methods should be called on the KarelPupper object.
        if response == "walk forward":
            self.pupper.walk_forward()
        elif response == "walk backward":
            self.pupper.walk_backward()
        elif response == "turn left":
            self.pupper.turn_left()
        elif response == "turn right":
            self.pupper.turn_right()
        elif response == "sit":
            self.pupper.sit()
        elif response == "stand":
            self.pupper.stand()
        elif response == "stop":
            self.pupper.stop()
        else:
            self.get_logger().warn(f"Unrecognized command: {response}")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    gpt4_conversation_node = GPT4ConversationNode()
    rclpy.spin(gpt4_conversation_node)

    # Clean up and shutdown
    gpt4_conversation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
