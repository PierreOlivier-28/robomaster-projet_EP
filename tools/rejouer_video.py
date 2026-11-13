"""Rejoue une video enregistree au labo : python tools/rejouer_video.py data/essai.mp4"""
import sys

import cv2


def main(chemin):
    cap = cv2.VideoCapture(chemin)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv2.imshow("rejeu", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1])
