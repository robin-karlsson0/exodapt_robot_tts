import rclpy
import riva.client
import riva.client.audio_io
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TtsRivaBridge(Node):
    '''
    '''

    def __init__(self):
        super().__init__('tts_riva_bridge')
        self.get_logger().info('TTS Riva Bridge Node Started')

        # Declare parameters with default 'robot girl' values
        self.declare_parameter('tts_voice_gender', 'female')
        self.declare_parameter('tts_voice_pitch', '3')
        self.declare_parameter('tts_voice_rate', '140%')
        self.declare_parameter('tts_voice_emotion', 'default')

        self.declare_parameter('tts_server_uri', '172.20.137.207:50051')
        self.declare_parameter('tts_topic', 'tts')

        server_uri = self.get_parameter('tts_server_uri').value
        tts_topic = self.get_parameter('tts_topic').value

        # Subscribe to the TTS topic
        # - All messages published will be voiced
        self.create_subscription(String, tts_topic, self.tts_callback, 10)

        # Initialize TTS service running on server
        auth = riva.client.Auth(uri=server_uri)
        self.riva_tts = riva.client.SpeechSynthesisService(auth)

        # Initialize sound stream
        output_device = None
        nchannels = 1
        sampwidth = 2
        sample_rate_hz = 44100
        self.sound_stream = riva.client.audio_io.SoundCallBack(
            output_device,
            nchannels=nchannels,
            sampwidth=sampwidth,
            framerate=sample_rate_hz)

        # Topic notifying other nodes like ASR if robot is speaking
        self.is_speaking_pub = self.create_publisher(
            Bool,
            '/tts_is_speaking',
            10,
        )

    def tts_callback(self, msg: String):
        '''
        Voices TTS messages.
        '''
        text = msg.data

        # Generate SSML and voice type according to config parameters
        gender, pitch, rate, emotion = self.get_voice_config()
        voice_name = 'English-US-RadTTS.Female-1'  # self.get_voice_name(gender, emotion)
        ssml = self.generate_ssml(text, pitch, rate)

        # Set 'is_speaking' state
        self.is_speaking_pub.publish(Bool(data=True))

        # Generate audio
        custom_dictionary_input = {}
        responses = self.riva_tts.synthesize_online(
            ssml,
            voice_name=voice_name,
            custom_dictionary=custom_dictionary_input,
        )

        for resp in responses:
            self.sound_stream(resp.audio)

        # Remove 'is_speaking' state
        self.is_speaking_pub.publish(Bool(data=False))

    def get_voice_name(self, gender: str, emotion: str) -> str:
        '''
        Returns the voice specification syntax for the Riva TTS service.
        '''
        voice_name = 'English-US-RadTTS.'

        if gender == 'female':
            voice_name += 'Female-'
            if emotion == 'default':
                voice_name += '1'
            if emotion == 'calm':
                voice_name += 'Calm'
            elif emotion == 'neutral':
                voice_name += 'Neutral'
            elif emotion == 'happy':
                voice_name += 'Happy'
            elif emotion == 'angry':
                voice_name += 'Angry'
            elif emotion == 'fearful':
                voice_name += 'Fearful'
            elif emotion == 'sad':
                voice_name += 'Sad'
            else:
                self.get_logger().warning(
                    'Invalid female emotion: {emotion}. Defaulting to \'default\'.'
                )
                voice_name += '1'

        elif gender == 'male':
            voice_name += 'Male-'
            if emotion == 'default':
                voice_name += '1'
            elif emotion == 'calm':
                voice_name += 'Calm'
            elif emotion == 'neutral':
                voice_name += 'Neutral'
            elif emotion == 'happy':
                voice_name += 'Happy'
            elif emotion == 'angry':
                voice_name += 'Angry'
            else:
                self.get_logger().warning(
                    'Invalid male emotion: {emotion}. Defaulting to \'default\'.'
                )
                voice_name += '1'

        else:
            self.get_logger().warning(
                'Invalid voice params: Defaulting to \'female default\'.')
            voice_name += 'Female-1'

        return voice_name

    def generate_ssml(self, text: str, pitch: str, rate: str) -> str:
        '''
        Returns a SSML templated text.
        '''
        ssml = f"""<speak><prosody pitch='{pitch}' rate='{rate}'>{text}</prosody></speak>"""
        return ssml

    def get_voice_config(self) -> tuple:
        '''
        Return current voice configuration parameters.
        '''
        gender = self.get_parameter('tts_voice_gender').value
        pitch = self.get_parameter('tts_voice_pitch').value
        rate = self.get_parameter('tts_voice_rate').value
        emotion = self.get_parameter('tts_voice_emotion').value
        return gender, pitch, rate, emotion


def main(args=None):
    rclpy.init(args=args)

    tts_riva_bridge = TtsRivaBridge()

    rclpy.spin(tts_riva_bridge)

    tts_riva_bridge.sound_stream.close()
    tts_riva_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
