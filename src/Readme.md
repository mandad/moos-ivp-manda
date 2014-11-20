
Autonomous Surface Vehicle Payload Autonomy Interface
=====================================================

Build Instructions
------------------

  1. Checkout the moos-ivp-extend tree
  
  svn co https://oceanai.mit.edu/svn/moos-ivp-extend/trunk moos-ivp-extend
  
  2. Navigate to the moos-ivp-extend folder
  
  cd moos-ivp-extend
  
  3. Check out the `src` folder of the trunk of the ASV repository (you may need to delete local .svn directories first)
  
  svn co https://mbogo.svn.cloudforge.com/seniorproject/trunk/src
  
  4. Run the `build.sh` script provided by moos-ivp-extend
  
  ./build.sh
  
  5. The iMOOSArduino and pMOOSArduinoTester applications should now be found in the `bin` folder
