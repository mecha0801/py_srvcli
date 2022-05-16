from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service') # 서비스 생성
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
                                       # 유형,          이름,              콜백                  을 정의함

    # 서비스 콜백 정의
    # 요청 데이터 수신 -> 합계 계산 -> 응답
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)   # Ros Client PYrhon 초기화

    minimal_service = MinimalService()  # 서비스 노드 생성 MinimalService 클래스를 인스턴스화

    rclpy.spin(minimal_service) # 노드를 회선시켜 콜백을 처리

    rclpy.shutdown()


if __name__ == '__main__':
    main()