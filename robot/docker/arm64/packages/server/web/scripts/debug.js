$(function () {
    $(document).on("click", "input", function () {
        var api = window.location.toString().replace("debug.html", "") + $(this).data("api");
        console.log(api);
        $.get(api, function (d) {
            $("#info").text("Status : " + ((d || {}).status || 1))
        })
    })
});
