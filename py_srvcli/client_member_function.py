# 클라이언트 메인에서 유일하게 중요한 차이점은 while loop
# loop는 시스템이 실행 중인 동안 서비스로부터 응답이 있는지 확인하기 위해 미래?를 확인
# 서비스에서 응답을 보낸 경우 결과는 로그 메시지에 기록됨
import sys  # 클라이언트 노드 코드가 명령줄 입력 인수에 액세스하기 위해 필요

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node): # 서비스 노드와 비슷한 유형의 이름

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):   # 클라이언트의 유형 및 이름과 일치하는 서비스를 초방 한번 사용할 수 있는지 확인함
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request() # 요청에 대한 정의

    def send_request(self): # 요청에 대한 주 정의
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()