# -*-coding:utf-8 -*-

import codecs

if __name__ == '__main__':
    reader = codecs.open('img_timestamps.txt','r',encoding='utf-8')
    writer = codecs.open('rgb_qi.txt','w',encoding='utf-8')
    # for line in reader:
    #     text = line.strip().split()
    #     word = text[1]
    #     writer.write(word+' '+'rgb/'+word+'.png')
    #     writer.write('\n')
    while lines:
    	lines = reader.readlines(38)
    	if not lines:
        	break
    	for line in lines:
        	print(line)
		