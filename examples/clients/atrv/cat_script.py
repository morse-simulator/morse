""" Simple script for the CAT AND MOUSE game tutorial

This will command the CAT, using two semantic cameras, to follow
after the MOUSE robot """

from pymorse import Morse

def is_mouse_visible(semantic_camera_stream):
    """ Read data from the semantic camera, and determine if a specific
    object is within the field of view of the robot """
    data = semantic_camera_stream.get()
    visible_objects = data['visible_objects']
    for visible_object in visible_objects:
        if visible_object['name'] == "MOUSE":
            return True
    return False

def main():
    """ Use the semantic cameras to locate the target and follow it """
    with Morse() as morse:
        semanticL = morse.cat.semanticL
        semanticR = morse.cat.semanticR
        motion = morse.cat.motion

        while True:
            mouse_seen_left = is_mouse_visible(semanticL)
            mouse_seen_right = is_mouse_visible(semanticR)
            if mouse_seen_left and mouse_seen_right:
                v_w = {"v": 2, "w": 0}
            elif mouse_seen_left:
                v_w = {"v": 1.5, "w": 1}
            elif mouse_seen_right:
                v_w = {"v": 1.5, "w": -1}
            else:
                v_w = {"v": 0, "w": -1}
            motion.publish(v_w)

if __name__ == "__main__":
    main()
