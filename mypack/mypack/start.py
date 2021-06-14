"""!
Запуск всех нод вместе в одном потоке
"""
from mypack.depth import * 
from mypack.draw import * 
from mypack.pcd_subscriber_node import *
import threading

def main(args = None):
    '''!
    Запуск всех нод
    @param args: аргументы
    @return: null
    '''
    rclpy.init(args = args)
    ## Инициализация нод
    detector = Detector()
    calculator = Depth_calculator()
    final = PCDListenerFinalPublisher()
    ## Добавление в один поток
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(calculator)
    executor.add_node(final)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    ## Запуск потока
    executor_thread.start()
    ## Создание rate объекта с частой 1Гц
    rate = detector.create_rate(1)

    try:
    	while rclpy.ok():
    		print(final.coordinates)
    		rate.sleep()
    except KeyboardInterrupt:
    	pass
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
	main() 
