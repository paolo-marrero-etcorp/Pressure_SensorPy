#!/usr/bin/python

import os
import sys
import re
import time
import subprocess
import logging

def main():

  # import pdb; pdb.set_trace();
  logging.basicConfig(filename='plc_wdog.log', format='%(asctime)s %(message)s', level=logging.DEBUG)
  wdog_ctr = 0
  while True:
    pid_found = False
    if os.path.isfile("g.txt"):
      cmd = 'rm g.txt'
      logging.info('Removed g.txt')
      os.system(cmd)
    cmd = 'ps -aef | grep plc_app > g.txt'
    os.system(cmd)
    try:
      f = open("g.txt", "r+")
    except IOError:
      logging.info('Cannot open g.txt')
    else:     
      i = 0   
      for line in f:
        print line
        m = re.search("./plc_app", line)
        if m:
          logging.info("Found at position %s in line %s" % str(m.start()), str(i))
          s = re.compile('\d+')
          p = s.search(line)
          cmux_pid = p.group()
          logging.info("PID of plc_app: %s" % cmux_pid)
          pid_found = True
        i += 1
      f.close(); 
      if pid_found == False:
        logging.warning("PID of plc_app not found restarting ....")
        subprocess.call("/home/src/Debug/plc_app")
        os.system(cmd)
        wdog_ctr += 1
      else:
        logging.info("plc_app is running ...")
      logging.info('done: rm g.txt')
      cmd = 'rm g.txt'
      os.system(cmd)
    logging.info("End of check loop sleep for 60 seconds")
    time.sleep(60)
    logging.info("watchdog counter = %s" % str(wdog_ctr))
    
if __name__ == '__main__':
  main()

