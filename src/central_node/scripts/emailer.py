#!/usr/bin/env python
import smtplib
import os
import re
# For guessing MIME type
import mimetypes

# Import the email modules we'll need
import email
import email.mime.application
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if 'Send email' in data.data:
        if len(data.data.split()) != 3 or not re.match(r"[^@]+@[^@]+\.[^@]+",  data.data.split()[2]):
            print ("bad addresss entered")
            return
        address = data.data.split()[2]
        send_from = 'photobot.picture@gmail.com'
        send_to = address
        subject = 'picture'
        text = 'Hello, it was great meeting you today at level 5 labs. Please find the photo we have taken together attached. Until next time!'


        msg = email.mime.Multipart.MIMEMultipart()
        msg['Subject'] = subject
        msg['From'] = send_from
        msg['To'] = send_to

        body = email.mime.Text.MIMEText(text)
        msg.attach(body)

        filename='/tmp/camera_image.jpeg'
        img_data = open(filename, 'rb').read()
        image = MIMEImage(img_data, name=os.path.basename(filename))
        msg.attach(image)

        server = smtplib.SMTP("smtp.gmail.com", 587)
        server.ehlo()
        server.starttls()
        server.login('photobot.picture@gmail.com', 'human444')
        server.sendmail(send_from, send_to, msg.as_string())
        picture = None #'pic.jpg'
#send_mail(send_from, send_to, subject, text, picture, server)
        server.close()


        
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('emailer', anonymous=True)
    rospy.Subscriber("event_output", String, callback)

# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()
