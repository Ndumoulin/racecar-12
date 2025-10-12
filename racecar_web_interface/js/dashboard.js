
document.addEventListener('DOMContentLoaded', function() {
    // Update navbar with stored credentials
    if (window.racecar && typeof window.racecar.updatenavbar === 'function') {
        window.racecar.updatenavbar();
    }

    // Attempt to connect to ROS using stored IP
    if (window.racecar && typeof window.racecar.connectROS === 'function') {
        window.racecar.connectROS();
    }
});