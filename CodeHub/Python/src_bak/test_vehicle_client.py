#!/usr/bin/env python3
# -*-coding:utf-8-*-

from twisted.internet.protocol import Factory
from twisted.protocols.basic import LineReceiver
from twisted.internet.endpoints import TCP4ServerEndpoint, connectProtocol


class LoggingProtocol(LineReceiver):
    def lineReceived(self, line):
        self.factory.fp.write(line + '\n')


class LogFactory(Factory):
    protocol = LoggingProtocol

    def __init__(self, filename):
        self.file = filename

    def startFactory(self):
        self.fp = open(self.file, 'a')

    def stopFactory(self):
        self.fp.close()