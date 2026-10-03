"""Generate the project's geometric AVM icon (optional development tool: Pillow)."""
from pathlib import Path
from PIL import Image, ImageDraw


def main():
    size = 256
    image = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle((12, 12, 244, 244), radius=38, fill='#142735')
    for box, color in [((34, 34, 122, 122), '#26a6be'), ((134, 34, 222, 122), '#477be0'),
                       ((34, 134, 122, 222), '#62b69f'), ((134, 134, 222, 222), '#e9b34e')]:
        draw.rounded_rectangle(box, radius=14, fill=color)
    draw.rounded_rectangle((101, 79, 155, 179), radius=17, fill='#edf4f8', outline='#142735', width=7)
    draw.rounded_rectangle((114, 96, 142, 117), radius=4, fill='#142735')
    destination = Path(__file__).resolve().parents[1]/'resources/video.ico'
    image.save(destination, format='ICO', sizes=[(32, 32), (64, 64), (128, 128), (256, 256)])
    print('Generated original geometric icon:', destination.name)


if __name__ == '__main__':
    main()
