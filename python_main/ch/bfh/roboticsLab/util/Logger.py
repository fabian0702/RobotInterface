import logging

"""
Logger class
@usageExample:
  import os
  from ch.bfh.roboticsLab.util import Logger
  logger = Logger.Logger(os.path.basename(__file__)).getInstance()
  logger.warning('My warning message...')
"""
class Logger():
  #TODO: may add a option to choose between a long or short log format
  def __init__(self, name:str, level=logging.DEBUG):
    self.logger = logging.getLogger(name)
    self.logger.setLevel(level)
    
    if not self.logger.handlers:
      # create console handler with a higher log level
      ch = logging.StreamHandler()
      ch.setLevel(logging.DEBUG)

      ch.setFormatter(self.CustomFormatter())
      
      self.logger.addHandler(ch)

  class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    blue = "\x1b[36;20m"
    green = "\x1b[32;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"

    formatString = '%(asctime)-15s %(name)-8s [%(levelname)-7s] %(message)s'
    FORMATS = {
      logging.DEBUG: grey + formatString + reset,
      logging.INFO: blue + formatString + reset,
      logging.WARNING: yellow + formatString + reset,
      logging.ERROR: red + formatString + reset,
      logging.CRITICAL: bold_red + formatString + reset
    }
    def format(self, record):
      log_fmt = self.FORMATS.get(record.levelno)
      formatter = logging.Formatter(log_fmt)
      return formatter.format(record)
  
  def getInstance(self):
    return self.logger
