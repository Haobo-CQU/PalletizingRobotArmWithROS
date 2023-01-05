#!/usr/bin/env python
# coding=utf-8
def get_const(color):

    config = {'0':{'lower':[40,43,46],'upper':[77,255,255]},
              '1':{'lower':[100,43,46],'upper':[124,255,255]},
              '2':{'lower':[156,43,46],'upper':[180,255,255],'lower2':[0,43,46],'upper2':[10,255,255]}}
    
    return config[color]
