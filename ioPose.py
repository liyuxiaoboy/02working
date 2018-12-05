# -*-coding:utf-8 -*-

import codecs
import re

if __name__ == '__main__':
    reader = codecs.open('corrected_poses.txt','r',encoding='utf-8')
    writer = codecs.open('rgb_qi.txt','w',encoding='utf-8')
    # for line in reader:
    #     text = line.strip().split()
    #     word = text[1]
    #     writer.write(word+' '+'rgb/'+word+'.png')
    #     writer.write('\n')
 
    while 1:
    	lines = reader.readlines(38)
    	if not lines:
    		break
    	for i in range(12):
    		line=lines[i]
        	if re.match('pcd_filename',line.strip()):
        		pcd_filename = line.strip().split('"')
        		word = pcd_filename[1]
        		writer.write(word+' ')
        	if re.match('timestamp',line.strip()):
        		timestamp = line.strip().split()
        		word = timestamp[1]
        		writer.write(word+' ')
        	if re.match('x:',line.strip()):
        		x_ = line.strip().split()
        		word = x_[1]
        		writer.write(word+' ')
        	if re.match('y:',line.strip()):
        		y_ = line.strip().split()
        		word = y_[1]
        		writer.write(word+' ')
        	if re.match('z:',line.strip()):
        		z_ = line.strip().split()
        		word = z_[1]
        		writer.write(word+'\n')
        	else:
        		continue
        
