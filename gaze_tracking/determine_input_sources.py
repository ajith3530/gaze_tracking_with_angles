import cv2

def get_available_video_sources():
    available_sources = []
    for i in range(10):  # You can adjust the range as per your requirement
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            break
        else:
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            codec = cap.get(cv2.CAP_PROP_FOURCC)
            available_sources.append((i, {"width": width, "height": height, "fps": fps, "codec": codec}))
        cap.release()
    return available_sources

if __name__ == "__main__":
    sources = get_available_video_sources()
    print("Available video sources:")
    for source in sources:
        print(f"Source Index: {source[0]}")
