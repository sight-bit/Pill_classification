from gtts import gTTS
import csv
import random
import time
import os
import io

while(True):
	f = io.open('Pill_list.csv', 'r', encoding='utf-8')
	r = list(csv.reader(f))
	detect = random.randint(1,20)

	print(r[detect][1])

	t = r[detect][1]
	f.close()


	text = str(t)
	# text = "이 약은 감기약입니다. 아침에 드세요."

	tts = gTTS(text=text, lang='ko')
	tts.save('med.mp3')

	os.system('vlc med.mp3 --play-and-exit')
	os.system('rm -f med.mp3')
	time.sleep(1)


