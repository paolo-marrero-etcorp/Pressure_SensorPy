from api_client import digital


def callback():
	print("Edge detect\n")

test_out = digital.DO("test_out")
test_in = digital.DI("test_in")
test_in.set_edge_detect(digital.RISING, callback)

polarity = 1

while(True):
	raw_input("Press a key\n")
	
	if polarity == 1:
		print("Setting high")
		test_out.set()
		polarity = 0
	else:
		print("Setting low")
		test_out.clear()
		polarity = 1