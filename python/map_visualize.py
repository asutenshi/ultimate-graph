import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation

def visualize_map_with_path(filename):
    # Читаем файл
    data = np.loadtxt(filename, dtype=int)

    # Утолщаем путь (значения 2) с помощью морфологического расширения
    path_mask = (data == 2)
    thick_path_mask = binary_dilation(path_mask, iterations=3)  # увеличить толщину линии в 3 раза
    
    # Создаём цветную карту
    # 0 = чёрный (препятствие)
    # 1 = белый (свободное пространство)
    # 2 = красный (путь)
    
    height, width = data.shape
    image = np.zeros((height, width, 3))
    
    # Заполняем цвета
    image[data == 0] = [0, 0, 0]       # чёрный
    image[data == 1] = [1, 1, 1]       # белый
    image[thick_path_mask] = [1, 0, 0] # красный (утолщённый путь)
    
    # Отображаем
    plt.figure(figsize=(12, 8))
    plt.imshow(image)
    plt.title('Map with Path')
    plt.axis('off')
    plt.tight_layout()
    plt.savefig('map_visualization.png', dpi=150, bbox_inches='tight')
    plt.show()

if __name__ == '__main__':
    visualize_map_with_path('../output/map_with_path.txt')
