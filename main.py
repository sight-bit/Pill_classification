from _serial import ser # serial
from _camera import change_cam, take_pic, up_led, down_led, up_cam, down_cam # gpio, os, time, cv
from _ttsService import speak # os, time
from readcsv import r,f # csv, io
from inference import infer # np, cv


if __name__ == '__main__':

    while True:

        if ser.readable():
            res = ser.readline().decode()

            if res == 'in\r\n':
                print('Pill in')

                # up camera
                frame_up = take_pic(up_cam, up_led)

                # down camera
                frame_down = take_pic(down_cam, down_led)

                # inference
                p_name, p_guide = infer(frame_up, frame_down)

                # tts service
                speak(p_name,p_guide)

                # down sevo 
                ser.write(str(p_guide).encode())
            
        
    ser.close()
    f.close()
    gp.cleanup()

