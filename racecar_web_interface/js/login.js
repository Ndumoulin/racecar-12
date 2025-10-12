// Login page behavior: save credentials and redirect to dashboard
document.addEventListener('DOMContentLoaded', function() {
    function $(id) { return document.getElementById(id); }
    var btnsubmit = $('submit-button');
    var statusbox = $('status-block');
    var ipInput = $('ip-adress');
    var usernameInput = $('username');

    if (btnsubmit && ipInput && usernameInput) {
        btnsubmit.addEventListener('click', function(event) {
            event.preventDefault();
            var ipadress = ipInput.value.trim();
            var username = usernameInput.value.trim();

            const ipRegex = /^(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)(\.(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)){3}$/;

            if (!ipRegex.test(ipadress)) {
                if (statusbox) statusbox.innerText = "Adresse IP invalide — format attendu: x.x.x.x (0-255)\n";
                return;
            }

            // Persist login info locally so other pages can read it
            try {
                localStorage.setItem('racecar_username', username);
                localStorage.setItem('racecar_ip', ipadress);
            } catch (e) {
                console.warn('localStorage unavailable', e);
            }

            // Redirect to dashboard; dashboard will use common.connectROS
            window.location.href = 'dashboard.html';
        });
    }
});