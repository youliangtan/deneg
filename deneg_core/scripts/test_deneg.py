from utils import State
from deneg_msgs.msg import Notify

s = State()
assert s.current() == Notify.JOIN
s.next()
assert s.current() == Notify.READY
s.next()
s.next()
assert s.current() == Notify.RANK

# throw bad states
s.leave()
assert s.current() == Notify.LEAVE
s.next()
assert s.current() == Notify.LEAVE
s.error()
assert s.current() == Notify.ERROR

# From start to end
s.reset()
assert s.current() == Notify.JOIN
s.next()
s.next()
s.next()
s.next()
assert s.current() == Notify.CONSENT
s.next()
assert s.current() == Notify.ASSIGNMENT
s.next()
assert s.current() == Notify.ASSIGNMENT

print("all passed")
