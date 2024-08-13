import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ######################
    #  Launch arguments
    ######################
    tts_voice_gender_arg = DeclareLaunchArgument(
        'tts_voice_gender',
        default_value='female',
        description='Synthesized voice gender {female, male}'
    )
    tts_voice_pitch_arg = DeclareLaunchArgument(
        'tts_voice_pitch',
        default_value='3',
        description='Synthesized voice pitch [-3, 3]'
    )
    tts_voice_rate_arg = DeclareLaunchArgument(
        'tts_voice_rate',
        default_value='140%',
        description='Synthesized voice rate [25%, 250%]'
    )
    tts_voice_emotion_arg = DeclareLaunchArgument(
        'tts_voice_emotion',
        default_value='3',
        description='Synthesized voice emotion {calm, neutral, happy, angry, fearful, sad}'
    )
    tts_server_uri_arg = DeclareLaunchArgument(
        'tts_server_uri',
        default_value='172.20.137.207:50051',
        description='Server URI to the Riva SDK TTS service'
    )
    tts_topic_arg = DeclareLaunchArgument(
        'tts_topic',
        default_value='tts',
        description='Topic with String messages to be voiced.'
    )
    
    return launch.LaunchDescription([
        # Launch arguments
        tts_voice_gender_arg,
        tts_voice_pitch_arg,
        tts_voice_rate_arg,
        tts_voice_emotion_arg,
        tts_server_uri_arg,
        tts_topic_arg,
        # Nodes
        launch_ros.actions.Node(
            package='tts_riva_bridge',
            executable='tts_riva_bridge',
            name='tts_riva_bridge',
            parameters=[{
                'tts_voice_gender': LaunchConfiguration('tts_voice_gender'),
                'tts_voice_pitch': LaunchConfiguration('tts_voice_pitch'),
                'tts_voice_rate': LaunchConfiguration('tts_voice_rate'),
                'tts_voice_emotion': LaunchConfiguration('tts_voice_emotion'),
                'tts_server_uri': LaunchConfiguration('tts_server_uri'),
                'tts_topic': LaunchConfiguration('tts_topic'),
            }]
        ),
    ])