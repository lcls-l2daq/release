#!/usr/bin/python
# ProcMgr.py - configure (start, stop, status) the DAQ processes

import os, sys, string, telnetlib, subprocess
import stat, errno, time
from re import search
from time import sleep

#
# The ProcMgr class maintains a dictionary with keys of
# the form "<host>:<uniqueid>".  The following helper functions
# are used to convert back and forth among <key>, <host>, and <uniqueid>.
#

#
# makekey - given <host> and <uniqueid>, generate a dictionary key
#
def makekey(host, uniqueid):
    return (host + ':' + uniqueid)
    
#
# key2host - given a dictionary key, get <host>
#
def key2host(key):
    return (key.split(':')[0])
    
#
# key2uniqueid - given a dictionary key, get <uniqueid>
#
def key2uniqueid(key):
    return (key.split(':')[1])

#
# mkdir_p - emulate mkdir -p
#
def mkdir_p(path):
    rv = 1
    try:
        os.makedirs(path)
    except OSError, exc:
        if exc.errno == errno.EEXIST:
            pass
        else: raise
    else:
        rv = 0
    return rv

#
# deduce_platform - deduce platform (-p) from contents of config file
#
# Returns: non-negative platform number, or -1 on error.
#
def deduce_platform(configfilename):
    # open the config file
    configfile = open(configfilename, 'r')
    rv=-1

    # for each line in the configuration...
    for line in configfile:
        # skip comment lines
        if (line[0]=='#'):
            continue

        xyz = line.split(" -p", 1)
        if len(xyz) == 2:
            zzz = xyz[1].split()[0]
            try:
                rv = string.atoi(zzz)
            except:
                rv = -1
            else:
                # success
                break

    if (rv < 0):
        rv = -1

    # close the config file
    configfile.close()

    return rv

    
class ProcMgr:

    # index into arrays managed by this class
    DICT_STATUS = 0
    DICT_PID = 1
    DICT_CMD = 2
    DICT_CTRL = 3
    DICT_LOG = 4
    DICT_PPID = 5
    DICT_FLAGS = 6
    DICT_GETID = 7

    # a managed executable can be in the following states
    STATUS_NOCONNECT = "NOCONNECT"
    STATUS_RUNNING = "RUNNING"
    STATUS_SHUTDOWN = "SHUTDOWN"
    STATUS_ERROR = "ERROR"

    # placeholder when PID is not valid
    STRING_NOPID = "-"

    # paths
    PATH_XTERM = "/usr/bin/xterm"
    PATH_TELNET = "/usr/bin/telnet"

    # messages expected from procServ
    MSG_BANNER_END = "server started at"
    MSG_ISSHUTDOWN = "is SHUT DOWN"
    MSG_RESTART = "new child"
    MSG_PROMPT = "\x0d\x0a> "
    MSG_SPAWN = "procServ: spawning daemon"

    # procmgr control port and log port initialized in __init__
    EXECMGRCTRL = -1
    EXECMGRLOG = -1

    # platform initialized in __init__
    PLATFORM = -1

    valid_flag_list = ['X', 'k', 's', '-']

    def __init__(self, configfilename, platform, baseport=29000):
        self.pid = self.STRING_NOPID
        self.ppid = self.STRING_NOPID
        self.getid = None
        self.telnet = telnetlib.Telnet()
        firstRemoteLine = True

        # configure the port numbers
        self.EXECMGRCTRL = baseport
        self.EXECMGRLOG = baseport+1

        # open the config file
        configfile = open(configfilename, 'r')

        # create new empty dictionary
        self.d = dict()

        if (platform == -1):
            # try to deduce platform from args used within config file
            platform = deduce_platform(configfilename)

        if (platform == -1):
            print 'ERR: platform not specified'
            return
        else:
            self.PLATFORM = platform

        if (self.PLATFORM > 0):
            self.EXECMGRLOG += (self.PLATFORM * 100)

        # assign ports dynamically
        nextCtrlPort = self.EXECMGRLOG + 1

        # for each line in the configuration...
        for line in configfile:
            line = line.strip()
            # skip empty lines
            if (line ==''):
                continue
            # skip comment lines
            if (line[0] =='#'):
                continue
            # extract fields from line
            fields = line.split(None, 3)
            if len(fields) != 4:
                # ...skip bad line
                print 'ERR: len = %d' % len(fields)
                print 'ERR: invalid config line: <<%s>>' % line.strip()
                continue

            self.host = fields[0]
            self.uniqueid = fields[1]

            # assign ports dynamically, in config file order
            self.ctrlport = str(nextCtrlPort)       # string
            self.logport = str(nextCtrlPort + 1)    # string
            nextCtrlPort = nextCtrlPort + 2         # integer

            self.flags = fields[2]
            for nextflag in self.flags:
                if (nextflag not in self.valid_flag_list):
                    print 'ERR: invalid flag:', nextflag
            self.cmd = fields[3].rstrip()
            self.pid = "-"
            self.ppid = "-"
            self.getid = "-"
            # open a connection to the logging port (procServ)
            try:
                self.telnet.open(self.host, self.logport)
            except:
                # telnet failed
                self.tmpstatus = self.STATUS_NOCONNECT
            else:
                # telnet succeeded: gather status from procServ banner
                ok = self.readLogPortBanner()
                if not ok:
                    # reading procServ banner failed
                    print "ERR: failed to read procServ banner for \'%s\' on host %s" \
                            % (self.uniqueid, self.host)
                # close connection to the logging port (procServ)
                self.telnet.close()

            if ((self.tmpstatus != self.STATUS_NOCONNECT) and \
                (self.tmpstatus != self.STATUS_ERROR) and \
                (self.getid != self.uniqueid)):
                print "ERR: found \'%s\', expected \'%s\' on host %s" % \
                    (self.getid, self.uniqueid, self.host)

            # add an entry to the dictionary
            key = makekey(self.host, self.uniqueid)
            self.d[key] = \
                [ self.tmpstatus, self.pid, self.cmd, self.ctrlport, self.logport, self.ppid, self.flags, self.getid]
                # DICT_STATUS  DICT_PID  DICT_CMD  DICT_CTRL      DICT_LOG      DICT_PPID  DICT_FLAGS  DICT_GETID

        # close the config file
        configfile.close()

    def readLogPortBanner(self):
        response = self.telnet.read_until(self.MSG_BANNER_END, 1)
        if not string.count(response, self.MSG_BANNER_END):
            self.tmpstatus = self.STATUS_ERROR
            return 0
        if search('SHUT DOWN', response):
            self.tmpstatus = self.STATUS_SHUTDOWN
            self.ppid = search('@@@ procServ server PID: ([0-9]*)', response).group(1)
            self.getid = search('@@@ Child \"(.*)\" start', response).group(1)
        else:
            self.tmpstatus = self.STATUS_RUNNING
            self.pid = search('@@@ Child \"(.*)\" PID: ([0-9]*)', response).group(2)
            self.getid = search('@@@ Child \"(.*)\" PID: ([0-9]*)', response).group(1)
            self.ppid = search('@@@ procServ server PID: ([0-9]*)', response).group(1)
        return 1

    #
    # show - call status() with an empty id_list
    #
    def show(self, verbose=0):
        return self.status([], verbose)

    #
    # status
    #
    def status(self, id_list, verbose=0):

        if self.isEmpty():
            if verbose:
                print "(configuration is empty)"
            return 1

        # print heading
        print "Host          UniqueID     Status     PID    PORT   Command+Args"

        # print contents of dictionary (sorted by key)
        for key in sorted(self.d.iterkeys()):

            if len(id_list) > 0:
                # if id_list is nonempty and UniqueID is not in it,
                # skip this entry
                if key2uniqueid(key) not in id_list:
                    continue

            if (self.d[key][self.DICT_STATUS] == self.STATUS_NOCONNECT):
                showId = key2uniqueid(key)
            else:
                showId = self.d[key][self.DICT_GETID]

            print "%-13s %-12s %-10s %-5s  %-5s  %s" % \
                    (key2host(key), showId, \
                    self.d[key][self.DICT_STATUS], \
                    self.d[key][self.DICT_PID], \
                    self.d[key][self.DICT_CTRL], \
                    self.d[key][self.DICT_CMD])
            if verbose:
                print "                                       PPID: %s  LOGPORT: %s" \
                        % (self.d[key][self.DICT_PPID], self.d[key][self.DICT_LOG])
        # done
        return 1

    #
    # restart
    #
    def restart(self, key, value, verbose=0):

        # open a connection to the procServ control port
        started = False
        connected = False
        telnetCount = 0
        host = key2host(key)
        while (not connected) and (telnetCount < 10):
            telnetCount = telnetCount + 1
            try:
                self.telnet.open(host, value[self.DICT_CTRL])
            except:
                sleep(.25)
            else:
                connected = True

        if connected:
            # wait for SHUT DOWN message
            response = self.telnet.read_until(self.MSG_ISSHUTDOWN, 1)
            if not string.count(response, self.MSG_ISSHUTDOWN):
                print 'ERR: no SHUT DOWN message in ',
                print 'response: <<%s>>' % response

            # send ^R to restart child process
            self.telnet.write("\x12");

            # wait for restart message
            response = self.telnet.read_until(self.MSG_RESTART, 1)
            if not string.count(response, self.MSG_RESTART):
                print 'ERR: no restart message... '
            else:
                started = True

                # close telnet connection
                self.telnet.close()
        else:
            print 'ERR: restart() telnet to %s port %s failed' % \
                (host, value[self.DICT_CTRL])

        return started

    #
    # startAll - call start() with an empty id_list
    #
    def startAll(self, verbose=0, logpathbase=None, coresize=0):
        return self.start([], verbose, logpathbase, coresize)

    #
    # start
    #
    def start(self, id_list, verbose=0, logpathbase=None, coresize=0):

        # create list of entries with X flag enabled (empty for now)
        Xlist = list()

        if self.isEmpty():
            # configuration is empty -- nothing to start
            if verbose:
                print 'startAll: empty configuration'
            return 1

        if (self.PLATFORM < 0) or (self.PLATFORM > 4):
            print 'platform %d not in range 0-4' % self.PLATFORM
            return 1

        # for redirecting to /dev/null
        nullOut = open(os.devnull, 'w')

        # create a dictionary mapping hosts to a set of start commands
        startdict = dict()
        for key, value in self.d.iteritems():
            if len(id_list) > 0:
                # if id_list is nonempty and UniqueID is not in it,
                # skip this entry
                if key2uniqueid(key) not in id_list:
                    continue
            if value[self.DICT_STATUS] == self.STATUS_NOCONNECT:
                starthost = key2host(key)
                if 'X' in value[self.DICT_FLAGS]:
                    Xlist.append([key, value])
                    waitflag = '--wait'
                else:
                    waitflag = ''

                if ('>' in value[self.DICT_CMD]) or (logpathbase == None) or (logpathbase == "/dev/null"):
                    redirect_string = ''
                else:
                    #
                    # Construct path similar to:
                    #
                    #  <logpath>/2009/08/21_10:35_atca01:opal1k.log
                    #
                    logpath = '%s/%s' % (logpathbase, time.strftime('%Y/%m'))
                    try:
                        mkdir_p(logpath)
                    except:
                        # mkdir
                        print 'ERR: mkdir %s failed' % logpath
                        redirect_string = ''
                    else:
                        os.chmod(logpath, stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)
                        time_string = time.strftime('%d_%H:%M:%S')
                        logfile = '%s/%s_%s.log' % (logpath, time_string, key)
                        if verbose:
                            print 'log file:', logfile
                        redirect_string = '>& %s' % logfile

                startcmd = \
                        '/reg/g/pcds/package/procServ-2.4.0/procServ --noautorestart --name %s %s --allow -l %s --coresize %d %s %s %s' % \
                        (key2uniqueid(key), \
                        waitflag, \
                        value[self.DICT_LOG], \
                        coresize, \
                        value[self.DICT_CTRL], \
                        value[self.DICT_CMD],
                        redirect_string)
                # is this host already in the dictionary?
                if starthost in startdict:
                    # yes: add to set of start commands
                    startdict[starthost].append(startcmd)
                else:
                    # no: initialize set of start commands
                    startdict[starthost] = [startcmd]
            elif value[self.DICT_STATUS] == self.STATUS_SHUTDOWN:
                # restart
                self.restart(key, value, verbose)

        # now use the newly created dictionary to run start command(s)
        # on each host

        for host, value in startdict.iteritems():

            if (host == 'localhost'):
                # process list of commands
                while len(value) > 0:
                    args = value.pop()
                    # send command
                    if verbose:
                        print 'Run locally: %s' % args

#                   yy = subprocess.Popen(args.split(), stdout=nullOut, stderr=nullOut, shell=False)
                    yy = subprocess.Popen(args, stdout=nullOut, stderr=nullOut, shell=True)
                    yy.wait()
                    if (yy.returncode != 0):
                        print 'ERR: failed to run %s (procServ returned %d)' % \
                            (args.split()[11], yy.returncode)
            else:
                # open a connection to the procmgr control port (procServ)
                try:
                    self.telnet.open(host, self.EXECMGRCTRL)
                except:
                    # telnet failed
                    print 'ERR: telnet to procmgr (%s port %d) failed' % \
                            (host, self.EXECMGRLOG)
                    print '>>> Please start the procServ process on host %s!' % host
                else:
                    # telnet succeeded

                    # send ^U followed by carriage return to safely reach the prompt
                    self.telnet.write("\x15\x0d");

                    # wait for prompt (procServ)
                    response = self.telnet.read_until(self.MSG_PROMPT, 2)
                    if not string.count(response, self.MSG_PROMPT):
                        print 'ERR: no prompt at %s port %s' % \
                            (key2host(key), self.EXECMGRCTRL)

                    # process list of commands
                    while len(value) > 0:
                        # send command
                        self.telnet.write('%s\n' % value.pop());
                        # wait for prompt
                        response = self.telnet.read_until(self.MSG_PROMPT, 2)
                        if not string.count(response, self.MSG_PROMPT):
                            print 'ERR: no prompt at %s port %s' % \
                                (host, self.EXECMGRCTRL)

                    # close telnet connection
                    self.telnet.close()

        for item in Xlist:

            # is xterm available?
            if os.path.exists(self.PATH_XTERM):
                # yes: spawn xterm
                args = [self.PATH_XTERM, "-T", item[0], \
                        "-e", self.PATH_TELNET, key2host(item[0]), \
                        item[1][self.DICT_CTRL]]
                subprocess.Popen(args)
            else:
                # no: say xterm not available
                print 'ERR: %s not available' % self.PATH_XTERM

        for item in Xlist:
            started = self.restart(item[0], item[1], verbose)
                        
        # done
        # cleanup
        nullOut.close()
        return 1

    #
    # isEmpty
    #
    def isEmpty(self):
        return (len(self.d) < 1)

    #
    # stopDictionary
    #
    def stopDictionary(self, sigchilddict, sigparentdict, verbose, sigdelay):
        rv = 0      # return value

        # for redirecting to /dev/null
        nullOut = open(os.devnull, 'w')

        # use the sigchild dictionary to run kill command on each host
        for host, value in sigchilddict.iteritems():

            if (host == 'localhost'):
                # local...
                # send the kill command (SIGINT)
                args = 'kill -2 %s' % value
                if verbose:
                    print 'localhost: ', args
                yy = subprocess.Popen(args.split(), stdout=nullOut, stderr=nullOut)
                yy.wait()
                if (yy.returncode != 0):
                    print 'ERR: failed to signal %d (kill returned %d)' % \
                        (value, yy.returncode)
                    rv = 1
            else:
                # remote...
                # open a connection to the procmgr control port (procServ)
                try:
                    self.telnet.open(host, self.EXECMGRCTRL)
                except:
                    # telnet failed
                    print 'ERR: telnet to procmgr failed'
                    print '>>> Please start the procServ process on host %s!' % host
                    rv = 1
                else:
                    # telnet succeeded

                    # send ^U followed by carriage return to safely reach the prompt
                    self.telnet.write("\x15\x0d");

                    # wait for prompt (procServ)
                    response = self.telnet.read_until(self.MSG_PROMPT, 2)
                    if not string.count(response, self.MSG_PROMPT):
                        print 'ERR: no prompt at %s port %s' % \
                            (key2host(key), self.EXECMGRCTRL)
                        rv = 1
                    if verbose:
                        print 'host %s: kill -2 %s' % (host, value)
                    # send the kill command (SIGINT)
                    self.telnet.write('kill -2 %s\n' % value);

                    # wait for prompt (procServ)
                    response = self.telnet.read_until(self.MSG_PROMPT, 2)
                    if not string.count(response, self.MSG_PROMPT):
                        print 'ERR: no prompt at %s port %s' % \
                            (key2host(key), self.EXECMGRCTRL)
                        rv = 1

                    # close telnet connection
                    self.telnet.close()

        # if 1 or more children were signalled, wait
        if len(sigchilddict) > 0:
            if sigdelay > 0:
                sleep(sigdelay)

        # use the sigparent dictionary to run kill command on each host
        for host, value in sigparentdict.iteritems():

            if (host == 'localhost'):
                # local...
                # send the kill command (SIGTERM)
                args = 'kill %s' % value
                if verbose:
                    print 'localhost: ', args
                yy = subprocess.Popen(args.split(), stdout=nullOut, stderr=nullOut)
                yy.wait()
                if (yy.returncode != 0):
                    print 'ERR: failed to signal %d (kill returned %d)' % \
                        (value, yy.returncode)
                    rv = 1
            else:
                # remote...
                # open a connection to the procmgr control port (procServ)
                try:
                    self.telnet.open(host, self.EXECMGRCTRL)
                except:
                    # telnet failed
                    print 'ERR: telnet to procmgr failed'
                    print '>>> Please start the procServ process on host %s!' % host
                    rv = 1
                else:
                    # telnet succeeded

                    # send ^U followed by carriage return to safely reach the prompt
                    self.telnet.write("\x15\x0d");

                    # wait for prompt (procServ)
                    response = self.telnet.read_until(self.MSG_PROMPT, 2)
                    if not string.count(response, self.MSG_PROMPT):
                        print 'ERR: no prompt at %s port %s' % \
                            (key2host(key), self.EXECMGRCTRL)
                        rv = 1
                    if verbose:
                        print 'host %s: kill %s' % (host, value)
                    # send the kill command (SIGTERM)
                    self.telnet.write('kill %s\n' % value);

                    # wait for prompt (procServ)
                    response = self.telnet.read_until(self.MSG_PROMPT, 2)
                    if not string.count(response, self.MSG_PROMPT):
                        print 'ERR: no prompt at %s port %s' % \
                            (key2host(key), self.EXECMGRCTRL)
                        rv = 1

                    # close telnet connection
                    self.telnet.close()

        # cleanup
        nullOut.close()

        return rv

    #
    # stopAll - call stop() with an empty id_list
    #
    def stopAll(self, verbose=0, sigdelay=1):
        return self.stop([], verbose, sigdelay)

    #
    # stop
    #
    def stop(self, id_list, verbose=0, sigdelay=1):
        rv = 0      # return value

        if self.isEmpty():
            # configuration is empty -- nothing to disconnect
            if verbose:
                print 'nothing to disconnect'
        else:
            #
            # create dictionaries mapping hosts to set of PIDs to be signalled
            #
            sigchilddict = dict()
            sigparentdict = dict()
            for key, value in self.d.iteritems():

                if len(id_list) > 0:
                    # if id_list is nonempty and UniqueID is not in it,
                    # skip this entry
                    if key2uniqueid(key) not in id_list:
                        continue
                else:
                    # if id_list is empty and 'k' flag is set,
                    # skip this entry
                    if 'k' in value[self.DICT_FLAGS]:
                        if verbose and value[self.DICT_STATUS] != self.STATUS_NOCONNECT:
                            print '\'%s\' not stopped: k flag is set in config file' % \
                                key2uniqueid(key)
                        continue
                
                # if process is RUNNING and has 's' in its flag, SIGINT it first
                if (value[self.DICT_STATUS] == self.STATUS_RUNNING) and \
                   ('s' in value[self.DICT_FLAGS]):
                    sigchildhost = key2host(key)
                    sigchildstring = value[self.DICT_PID]
                    if sigchildhost not in sigchilddict:
                        # first PID for this host
                        sigchilddict[sigchildhost] = sigchildstring
                    else:
                        # not the first PID for this host
                        sigchilddict[sigchildhost] += (' ' + sigchildstring)

                # if process is not NOCONNECT, SIGTERM its parent (procServ)
                if value[self.DICT_STATUS] != self.STATUS_NOCONNECT:
                    sigparenthost = key2host(key)
                    sigparentstring = value[self.DICT_PPID]
                    if sigparenthost not in sigparentdict:
                        # first PID for this host
                        sigparentdict[sigparenthost] = sigparentstring
                    else:
                        # not the first PID for this host
                        sigparentdict[sigparenthost] += (' ' + sigparentstring)

            rv = self.stopDictionary(sigchilddict, sigparentdict,
                                     verbose, sigdelay)

        # done
        return rv

#
# main
#
if __name__ == '__main__':
    basename = os.path.splitext(os.path.basename(sys.argv[0]))[0]
    usage = "Usage: %s configfile" % basename
    if len(sys.argv) != 2:
        print usage
        sys.exit(1)

    # collect the status, reading from the config file
    print '-------- calling ProcMgr(%s)' % sys.argv[1]
    try:
        procMgr = ProcMgr(sys.argv[1])
    except IOError:
        print "%s: error while accessing %s" % (sys.argv[0], sys.argv[1])
        sys.exit(1)

    # error check
    if procMgr.isEmpty():
        print 'configuration is empty.  exiting now'
        sys.exit(1)

    # show the status
    print '-------- calling show(0)'
    procMgr.show(0)

    # stop all
    print '-------- calling stopAll()'
    procMgr.stopAll()

    # delete the previous status (is this right?)
    del procMgr

    # collect the status again
    print '-------- calling procMgr(%s)' % sys.argv[1]
    try:
        procMgr = procMgr(sys.argv[1])
    except IOError:
        print "%s: error while accessing %s" % (sys.argv[0], sys.argv[1])
        sys.exit(1)

    # error check
    if procMgr.isEmpty():
        print 'configuration is empty.  exiting now'
        sys.exit(1)

    # show the status again
    print '-------- calling show(0)'
    procMgr.show(0)

    # start all
    print '-------- calling startAll(1)'
    procMgr.startAll(1)

    # delete the previous status (is this right?)
    del procMgr

    # collect the status again
    print '-------- calling procMgr(%s)' % sys.argv[1]
    try:
        procMgr = procMgr(sys.argv[1])
    except IOError:
        print "%s: error while accessing %s" % (sys.argv[0], sys.argv[1])
        sys.exit(1)

    # show the status again
    print '-------- calling show(0)'
    procMgr.show(0)

    print '-------- done'
