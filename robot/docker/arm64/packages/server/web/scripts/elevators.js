$(function () {
    $("#submitbtn").click(() => {
        $.get("/elevator/call", {
            bid: $("#bid").val(),
            src: $("#src").val(),
            dst: $("#dst").val()
        }, (d) => {
            console.log(d)
            $("#info").text(Date.now() + " : " + $("#info").text() + "\n" + d.msg);
        })
    })
});
