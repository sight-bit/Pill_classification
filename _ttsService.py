from gtts import gTTS 
import os
import time

pg_dict={'0':' 아침', '1':' 점심', '2':' 저녁'}

def speak(pn, pg):
    text = pn +' 입니다.'+ pg_dict[pg]+'에 드세요.'

    print(text)

    tts = gTTS(text=text, lang='ko')
    tts.save('med.mp3')

    os.system('cvlc med.mp3 --play-and-exit')
    os.system('rm -f med.mp3')
    time.sleep(1) 
    


