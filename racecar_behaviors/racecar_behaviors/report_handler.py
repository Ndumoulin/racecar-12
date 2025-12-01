#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from racecar_interfaces.srv import ReportDebris
import os
from datetime import datetime

class ReportHandler(Node):
    def __init__(self):
        super().__init__('report_handler')
        self.srv = self.create_service(ReportDebris, 'report_debris', self.report_callback)
        self.reports = []
        
        # File setup
        self.report_file = os.path.expanduser('~/debris_report.txt')
        with open(self.report_file, 'a') as f:
            f.write(f"\n--- Session Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")
            
        self.get_logger().info(f'Report Handler Service Ready. Writing to {self.report_file}')

    def report_callback(self, request, response):
        try:
            report_entry = {
                'photo': request.photo_filename,
                'position': (request.position.x, request.position.y)
            }
            self.reports.append(report_entry)
            
            # Write to file
            log_line = f"Photo: {request.photo_filename}, Position: ({request.position.x:.2f}, {request.position.y:.2f})\n"
            with open(self.report_file, 'a') as f:
                f.write(log_line)
            
            self.get_logger().info(f"New Report Logged: {log_line.strip()}")
            
            response.success = True
            response.message = f"Report stored and written to file. Total: {len(self.reports)}"
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
