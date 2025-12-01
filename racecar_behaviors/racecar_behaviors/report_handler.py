#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from racecar_interfaces.srv import ReportDebris

class ReportHandler(Node):
    def __init__(self):
        super().__init__('report_handler')
        self.srv = self.create_service(ReportDebris, 'report_debris', self.report_callback)
        self.reports = []
        self.get_logger().info('Report Handler Service Ready.')

    def report_callback(self, request, response):
        try:
            report_entry = {
                'photo': request.photo_filename,
                'position': (request.position.x, request.position.y)
            }
            self.reports.append(report_entry)
            
            self.get_logger().info(f"New Report Received: Photo={request.photo_filename}, Pos={report_entry['position']}")
            self.get_logger().info(f"Total Reports: {len(self.reports)}")
            
            response.success = True
            response.message = f"Report stored. Total: {len(self.reports)}"
        except Exception as e:
            self.get_logger().error(f"Failed to store report: {e}")
            response.success = False
            response.message = str(e)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ReportHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
