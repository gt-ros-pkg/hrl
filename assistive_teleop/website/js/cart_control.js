function cart_init(){
    };

$(function(){
    cart_arm();    
    });    

function pub_cart_twist(trans, rot){
    tws = window.TwistStamped;
    tws.header.frame_id = $('#cart_frame_select').val()
    tws.twist.linear.x = trans[0];
    tws.twist.linear.y = trans[1];
    tws.twist.linear.z = trans[2];
    tws.twist.angular.x = rot[0];
    tws.twist.angular.y = rot[1];
    tws.twist.angular.z = rot[2];
    node.publish('cart/commands','geometry_msgs/TwistStamped', json(tws));
    };


function cart_arm(){
    $('#bpd_cart_trans').find(':button').unbind('.rfh');
    $('#bpd_cart_trans #b9').show().bind('click.rfh', function(e){
        pub_cart_twist([scales[window.arm()[0]+'arm'],0,0],[0,0,0]);
    });
    $('#bpd_cart_trans #b8').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,scales[window.arm()[0]+'arm']],[0,0,0]);
    });
    $('#bpd_cart_trans #b7').show().bind('click.rfh', function(e){
        pub_cart_twist([-scales[window.arm()[0]+'arm'],0,0],[0,0,0]);
    });
    $('#bpd_cart_trans #b6').show().bind('click.rfh', function(e){
        pub_cart_twist([0,-scales[window.arm()[0]+'arm'],0],[0,0,0]);
    });
    $('#bpd_cart_trans #b5').hide()
    $('#bpd_cart_trans #b4').show().bind('click.rfh', function(e){
        pub_cart_twist([0,scales[window.arm()[0]+'arm'],0],[0,0,0]);
    });
    $('#bpd_cart_trans #b3').hide();
    $('#bpd_cart_trans #b2').show().bind('click.rfh', function(e){
        pub_cart_twist([-scales[window.arm()[0]+'arm'],0,0],[0,0,0]);
    });
    $('#bpd_cart_trans #b1').hide();
    
    $('#bpd_cart_rot').find(':button').unbind('.rfh');
    $('#bpd_cart_rot #b9').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[0,0,-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b8').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b7').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b6').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b5').hide()
    $('#bpd_cart_rot #b4').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b3').hide();
    $('#bpd_cart_rot #b2').show().bind('click.rfh', function(e){
        pub_cart_twist([0,0,0],[-scales[window.arm()[0]+'_cart_rot'],0,0]);
    });
    $('#bpd_cart_rot #b1').hide();
};
