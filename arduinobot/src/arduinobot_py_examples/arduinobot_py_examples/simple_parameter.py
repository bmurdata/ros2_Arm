import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param",28)
        self.declare_parameter("simple_string_parameter","Brian")
        # Function that is called whnen a parameter is changed
        self.add_on_set_parameters_callback(self.paramChangeCallback)
    # Change parameter when called
    def paramChangeCallback(self,params):
        result=SetParametersResult()
        # Log result to terminal
        for param in params:
            if param.name=="simple_int_param" and param.type_==Parameter.Type.INTEGER:
                self.get_logger().info(f"Param {param.name} changed! New Value: {param.value}")
                result.successful=True
            if param.name=="simple_string_parameter" and param.type_==Parameter.Type.STRING:
                self.get_logger().info(f"Param {param.name} changed! New Value: {param.value}")
                result.successful=True
        return result

def main():
    rclpy.init()
    simple_parameter=SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    main()