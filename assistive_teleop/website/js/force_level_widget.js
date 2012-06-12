
ft_viewer_html = '<table><tr><td>\
            <table border=1 style="height:450px; width:10px">\
        <tr id="ft_ref_danger" style="height:33%;background-color:red">\
          <td id="ft_ref_danger_label">[Danger Force Level]</td>\
        </tr><tr id="ft_ref_act" style="height:50%;background-color:green">\
          <td id="ft_ref_act_label">[Activity Force Level]</td>\
        </tr><tr id="ft_ref_null" style="height:100%;background-color:gray">\
         <td><div id="ft_readout"></div></td></tr></table>\
        </td><td><div id="ft_bar_wrapper">\
          <div id="ft_bar_value"></div></div></td></tr></table>'

$(function(){
$.widget("rfh.ft_viewer",{
    options:{
        clear:null,
        yellow_pct:50,
        max_force:15
    },
    _create: function(){
        this.element.html(ft_viewer_html);
        window.node.subscribe('/wt_force_out_throttle',
                        function(ws){this._refresh(ws)})
        if (window.get_param_free){
            window.get_param_free = false;
            node.rosjs.callService('/rosbridge/get_param',
                                   '["face_adls_manager"]',
              function(params){
                  var dt = params['dangerous_force_thresh'];
                  var at = params['activity_force_thresh'];
                  var danger_pct=((this.max_force-dt)/this.max_force)*100;
                  var act_pct = ((this.max_force-at)/this.max_force)*100-danger_pct;
                  $("#ft_ref_danger").height(danger_pct.toString()+'%');
                  $("#ft_ref_danger_label").text("Danger\r\n >"+params.dangerous_force_thresh.toString()+" N");
                  $("#ft_ref_act").height(act_pct.toString()+'%');
                  $("#ft_ref_act_label").text("Activity\r\n  >"+params.activity_force_thresh.toString()+ "N");
                  window.get_param_free=true;
          })} else {
              set_timeout('this.create()');
          };

    },
    _destroy: function(){
        this.element.html('');
    },
    _refresh: function(ws){
       mag = Math.sqrt(Math.pow(ws.wrench.force.x,2) + 
                       Math.pow(ws.wrench.force.y,2) + 
                       Math.pow(ws.wrench.force.z,2))
       var pct = Math.min(mag, 15)/15;
       $('#ft_readout').html('<p>'+mag.toFixed(1)+' N </p>')
       var g = r = "FF";
       if (pct > this.yellow_pct){
           g = Math.round((255*(1-(pct-this.yellow_pct)/(1-this.yellow_pct)))).toString(16);
           if (g.length==1){g="0"+g};
       };
       if (pct < this.yellow_pct){
           r = (Math.round(255*(pct/this.yellow_pct))).toString(16);
           if (r.length==1){ r="0"+r};
       };
       color = "#"+r+g+'00';
       $('#ft_bar_wrapper').css('background-color', color);
       $('#ft_bar_value').css('height', Math.round(100*(1-pct)).toString()+'%');
        
    },
});


});

