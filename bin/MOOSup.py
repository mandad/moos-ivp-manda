#
#   MOOSup.py
#   Authors: Erin Fischell and Alon Yaari
#   Date: May 2013
#   Laboratory for Autonomous Marine Sensing Systems, MIT
#
#   Desc: Script to package a MOOS working set and upload the resulting tarfile to a MOOS submission server
#   This script should be located along with a submitCode.sh script
#
#   Output: a tarfile with naming:  ../<teamName>_<setName>.tar.gz
#
import sys
import os


class SubmitInfo:

    def __init__(self, in_data):
        self.data = {}
        for row in in_data:
            attr = str(row[0])
            val = row[1]
            self.data[attr] = val
        self.data['moosDir'] = os.getcwd().split('/')[-1]
        self.data['moosPath'] = '../' + os.getcwd().split('/')[-1]

    def __repr__(self):
        return repr(self.data)

    def __getitem__(self, item):
        return self.data[item]

    def __setitem__(self, key, item):
        self.data[key] = item

    def keys(self):
        return self.data.keys()


def getVarsFromFile(fileName):
    f = open(fileName, 'r')
    attrList = []
    for line in f:
        if line[0] != '#':
            items = line.split('=')
            key = items[0]
            val = items[1].split('\n')[0]
            attrList.append([key, val])
    existFields = SubmitInfo(attrList)
    return existFields


def checkOldSubmit():
    # Check to see if there is an existing temporary file
    #       - Will be put it in data/
    os.system('mkdir -p data')
    filename = 'data/submitData.txt'
    try:
        open(filename, 'r')
        out = getVarsFromFile(filename)
    except IOError:
        out = None
        pass
    return out


def getInputs(useOld=True, useScp=True):
    oldInput = checkOldSubmit()
    email = None
    lastName = None
    firstName = None
    teamName = None
    url = None
    if useOld and oldInput is not None:
        if 'teamName' in oldInput.keys():
            teamName = oldInput['teamName']
        if 'email' in oldInput.keys():
            email = oldInput['email']
        if 'firstName' in oldInput.keys():
            firstName = oldInput['firstName']
        if 'lastName' in oldInput.keys():
            lastName = oldInput['lastName']
        if 'url' in oldInput.keys():
            url = oldInput['url']
    if teamName is None:
        teamName = raw_input('Team name: ')
    if email is None:
        email = raw_input('Team contact person\'s E-mail address: ')
    if firstName is None:
        firstName = raw_input('Please input your team contact person\'s first name: ')
    if lastName is None:
        lastName = raw_input('Please input your team contact person\'s last name: ')
    if url is None and useScp:
        url = raw_input('IP or url of MOOS submission server (e.g., 128.30.29.31 or moosup.csail.mit.edu): ')

    # Establish proper mission directory
    missionDir = None
    bValidDir = False
    while not bValidDir:
        print '\nPlease identify the mission directory to be considered with this submission. Choices are: '
        myList = os.listdir('missions/')
        for el in myList:
            if el[0] != '.':
                print "   " + el
        missionDir = raw_input('Enter the directory name from the list: ')
        bValidDir = os.path.isdir('missions/' + missionDir)
        if not bValidDir:
            print missionDir + ' is an invalid mission directory. Enter a name from the list or ctrl-c to exit.'
    storeFile = open('data/submitData.txt', 'w')
    if teamName:
        print >> storeFile, 'teamName=' + str(teamName)
    if email:
        print >> storeFile, 'email=' + str(email)
    if firstName:
        print >> storeFile, 'firstName=' + str(firstName)
    if lastName:
        print >> storeFile, 'lastName=' + str(lastName)
    if url:
        print >> storeFile, 'url=' + str(url)
    storeFile.close()

    # Copy the missionDir specified to missions/current
    os.system('mkdir missions/current; cp -r missions/' + missionDir + '/*.bhv missions/' +
              missionDir + '/*.moos missions/' + missionDir + '/*.sh missions/current/')
    existFields = SubmitInfo([['teamName', teamName],
                              ['passcode', ''],
                              ['email', email],
                              ['missionDir', missionDir],
                              ['lastName', lastName],
                              ['firstName', firstName],
                              ['url', url]])
    return existFields


def confirmMOOSStructure(moosDir):
    #confirm that things are in the correct places:
    # TODO: add other path existence checks here.
    good = False
    srcExists = os.path.isdir(moosDir + '/src')
    cMakeSrcExists = os.path.isfile(moosDir + '/src/CMakeLists.txt')
    missionFolderExists = os.path.isdir(moosDir + '/missions/current')
    buildExists = os.path.isfile(moosDir + '/build.sh')
    numMoos = 0
    moosFileGood = False
    for myFile in os.listdir(str(moosDir + '/missions/current/')):
        if 'moos' in myFile:
            numMoos += 1
    if numMoos:
        moosFileGood = True
    if srcExists and missionFolderExists and cMakeSrcExists and moosFileGood:
        print '\nStructure Appears to be Correct...'
        good = True
    if not srcExists:
        raise Exception('Error in the structure of your moos-ivp-extend tree. You must have a src/ directory')
    if not missionFolderExists:
        raise Exception('Error in the structure of your moos-ivp-extend tree. You must have a missions/ directory')
    if not cMakeSrcExists:
        raise Exception('Error in the structure of your moos-ivp-extend tree. You must have a file src/CMakeLists.txt')
    if not buildExists:
        raise Exception('Error in the structure of your moos-ivp-extend tree. '
                        'You must have a build.sh file in your moos-ivp-extend directory.')
    if not moosFileGood:
        raise Exception('Error in the structure of your moos-ivp-extend tree. '
                        'You must have at least one .moos file in the missions directory you specify.')
    return good


def submitCode(existFields, lab_number, run_scp, baseDir):
    # First, tar up the file
    #       - Note that in the actual version directory will be replaced w/ moos-ivp-external or whatever
    teamName = existFields['teamName']
    moosDir = existFields['moosDir']
    moosPath = existFields['moosPath']
    tarfile = '../' + teamName + '_set' + str(lab_number) + '.tar.gz'
    #confirm that the required structure is there (there must be a src directory and a missions directory)
    # TODO: Do something with the confirmation of the MOOS structure
    good = confirmMOOSStructure(moosPath)
    print '\nTarring your directory now...'
    # TODO: Construct the excludes from a list of directories/files to exclude
    tarCmd = 'tar --directory=' + baseDir + '/.. --exclude \".svn\" --exclude \"' + \
              moosDir + '/bin/*\" ' + '--exclude \"' + moosDir + '/build/*\" ' + \
              '--exclude \"' + moosDir + '/lib/*\" '+'--exclude \"' + moosDir + \
              '/missions/*/LOG*\" -czf ' + tarfile + ' ' + moosDir
    os.system(tarCmd)
    # Now that directory is zipped up, submit it:
    if run_scp:
        print "Running scp on the tarfile..."
        runSCP(tarfile, existFields['url'])


def runSCP(tarFileName, url):
    # Run scp, using the teamName and a static password already entered.  This will send the zipped folder.
    # TODO: implement the scp function in this@url:
    scpCmd = 'scp ' + tarFileName + ' student@' + url + ': <<student'
    print scpCmd
    success = os.system(scpCmd)
    if success == 0:
        return True
    else:
        print "Error automatically sending your file via scp, verify your internet connection and try again."
        return False
    pass


def main(run_scp, baseDir):
    os.chdir(baseDir)
    existFields = getInputs(useScp=run_scp)

    # Ask if the resulting values are correct:
    good = False
    while not good:
        print '\nUsing the following fields:\n' + \
              '   Missions Directory: ' + str(existFields['missionDir']) + '\n' + \
              '   MOOS Directory:     ' + str(existFields['moosDir']) + '\n' + \
              '   Contact Email:      ' + str(existFields['email']) + '\n' + \
              '   Team Name:          ' + str(existFields['teamName'])
        new = raw_input('Are these Correct? [Y]/N ')
        if new == 'N' or new == 'n':
            existFields = getInputs(False)
        else:
            good = True

    # Request set name
    setName = raw_input('\nPlease Input the set ID: ')
    start3 = setName[:3]
    if start3 in 'set Set SET':
        setName = setName[3:]
    # Run submit code
    submitCode(existFields, setName, run_scp, baseDir)
    #clean up
    os.system('rm -r missions/current')
    print '\nDone!'


def Usage():
    print '   Usage:       MOOSUp [--tar_only]\n\n' + \
          '   Description: Packages a MOOS working directory as a compressed tarfile (.tar.gz extension)\n' + \
          '                and uploads the resulting file to a MOOS submission server via scp.\n' + \
          '                --help        Also -h. This usage list\n' + \
          '                --tar_only    Also -t. Completes the compressed tar of the MOOS working\n' + \
          '                              directory but does not upload the file via scp.\n' + \
          '   To use:      Navigate to the root of your moos-ivp-XXX directory and enter:\n' + \
          '                $ python MOOSUp\n'


#------------------------------------------------
# initial code that is run:

baseDir = os.getcwd()
run_scp = True
if (len(sys.argv) > 1):
    if sys.argv[1] in '-tar_only --tar_only -t':
        print 'Tarfile only; no submission of resulting tarfile.'
        main(False, baseDir)
    elif sys.argv[1] in '-h --help':
        Usage()
    else:
        print 'Unrecognized flag: only allowable are -h (help) or -tar_only (tar only, no scp)\n'
        Usage()
else:
    main(True, baseDir)


