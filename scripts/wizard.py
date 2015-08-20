import socket
import random
import time



def main():
   # Create the send port
   wizard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

   # Get the IP and port of the target
   sendIP = raw_input("Enter the IP of the puppet: ").strip()
   sendPort = int(raw_input("Enter the port of the puppet: ").strip())

   # Create the connection
   wizard.connect((sendIP, sendPort))

   itemList = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
   used = []

   size = 1024

   quit = False
   while not quit:

      # Begin the experiment
      print "New experiment"

      used = []

      # Press enter when it is time to prompt for answers (3 min mark)
      print "Press enter to wave"
      ignore = raw_input()

      wizard.send("wave 0")

      # Press enter when it is time to prompt for answers (3 min mark)
      print "Press enter to prompt for answers at the three minute mark"
      ignore = raw_input()

      # Send command for prompt
      wizard.send("start 0")
      #tts("Hello, you have two minutes left. Are you ready to start recording answers?")

      # Press enter when ready
      ignore = raw_input("Enter to start recording answers")

      # Start item collection
      for i in range(1, 6):

         print "This is item number " + str (i)

         # Failure loop
         loop = True
         while loop:
            # Send command for item prompt
            wizard.send("prompt " + str(i))
            #tts("What is your choice for item number " + str(i))

            # Wait for continue message
            ignore = wizard.recv(size)

            # Get the item number
            valid = False
            while not valid:
               itemNum = -1
               userInput = raw_input("\nEnter item number, or enter 0 to prompt again: "
               if is_number(userInput):
                  itemNum = int(userInput).strip())

               if itemNum >= 0 and itemNum < 11:
                  valid = True

               else:
                  print "That wasn't a correct item, the item must be between zero and ten"

            # Check for failure
            if itemNum == 0:
               # Send the failure message
               wizard.send("fail 0")
               print "Fail message given"
               # Wait for continue message
               ignore = wizard.recv(size)
               # Make a brief pause
               time.sleep(1)
               continue
            else:
               loop = False

            used.append(itemNum)

            # If this is test 1,2 or 4, give the right answer
            if i == 1 or i == 2 or i == 4:
               #used.append(itemNum)
               print "Correct item used"

            # Else pick a random wrong answer
            else:
               flag = True
               while flag:
                  itemNum = random.randrange(1,10)
                  if itemNum not in used:
                     flag = False

               used.append(itemNum)
               print "Wrong item used"

         loop = True
         while loop:
            # Send confirm message
            wizard.send("confirm " + str(itemNum))
            #tts("Did you choose " + items[itemNum])

            # Wait for continue message
            ignore = wizard.recv(size)

            # Wait to give response
            check = raw_input("\nEnter when response given, or 0 to prompt again: ") .strip()

            # Check for failure
            if check == '0':
               # Send the failure message
               wizard.send("fail 0")
               print "Fail message given"
               # Wait for continue message
               ignore = wizard.recv(size)
               # Make a brief pause
               time.sleep(1)
               continue
            else:
               loop = False

            # If this is test 1,2,4 then give the happy response
            if i == 1 or i == 2 or i == 4:
               wizard.send("happy 0")
               print "Happy face shown"

               # Wait for continue message
               ignore = wizard.recv(size)

            # Else give the sad response based on the trial number
            else:
               wizard.send("sad " + str(i))
               print "Sad face shown"

               # Wait for continue message
               ignore = wizard.recv(size)

      # Say goodbye
      time.sleep(3)
      wizard.send("bye 0")


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

if __name__ == '__main__':
   main()